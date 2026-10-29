"""Planificatorul Greedy pentru S1 (reproducere), S2 (turn) și S3 (sortare).

Planul se construiește pe o copie a lumii: fiecare mutare aleasă e aplicată imediat, ca următoarea să vadă
efectul ei (dezgroparea unui cub, eliberarea celulei țintă). Reguli comune:
- cuburile deja la locul lor (prefixul corect al turnului, celulele S1 deja corecte, cuburile din zone)
  nu sunt atinse;
- un cub îngropat se dezgroapă mutând cuburile de deasupra pe celule libere din `staging_locations`;
- nicio atribuire dublă: fiecare mutare ia vârful real al stivei din lumea simulată a planului;
- obiectivele neîndeplinite au motive explicite („lipsește 1 cub roșu”, „nivel peste limită”).

Orchestrarea execută doar prima mutare, reobservă și replanifică (buclă închisă).
"""

from __future__ import annotations

from typing import Dict, FrozenSet, List, Optional, Set, Tuple

from brain.configuration.models import SortMapConfig, SystemConfig
from brain.domain.cube import KNOWN_COLORS, Color, TopFace
from brain.domain.grasp import Clearances, Footprint, clearances, footprint_from_uv, plan_grasp, plan_release
from brain.domain.plan import CubeRef, Move, PickAction, PlaceAction, Plan, SlotRef, correct_parallax
from brain.domain.scene_state import SceneState
from brain.planning.cost import CostModel
from brain.planning.goals import Goal, ReproduceGoal, SortGoal, TowerGoal, color_name, missing_text
from brain.planning.world import PlanCube, PlanningWorld, _slot_key, grid_sizes_from_config

MAX_STEPS = 120  # siguranță: nicio sarcină reală nu are nevoie de atâtea mutări


class Planner:
    def __init__(self, system: SystemConfig, sort_map: SortMapConfig, cost: CostModel) -> None:
        self.system = system
        self.sort_map = sort_map
        self.cost = cost

    def world_for(self, goal: Goal, state: SceneState) -> PlanningWorld:
        """Șabloanele active + punctele tampon + (doar la S3) zonele hărții."""
        planning = self.system.planning
        points = list(planning.buffer_points)
        if isinstance(goal, SortGoal):
            points += [z.name for z in self.sort_map.zones]
        active = {name: self.system.locations[name] for name in planning.active_locations}
        return PlanningWorld.from_scene(state, grid_sizes_from_config(active), points, planning.min_confidence)

    def plan(
        self,
        goal: Goal,
        state: SceneState,
        avoid_sources: FrozenSet[SlotRef] = frozenset(),
        avoid_targets: FrozenSet[SlotRef] = frozenset(),
        start: Optional[SlotRef] = None,
    ) -> Plan:
        builder = _Builder(self, self.world_for(goal, state), avoid_sources, avoid_targets, start)
        if isinstance(goal, TowerGoal):
            builder.tower(goal)
        elif isinstance(goal, ReproduceGoal):
            builder.reproduce(goal)
        elif isinstance(goal, SortGoal):
            builder.sort()
        else:  # pragma: no cover - tipurile sunt închise
            raise TypeError("obiectiv necunoscut: {!r}".format(goal))
        return builder.result()


class _Builder:
    def __init__(
        self,
        planner: Planner,
        world: PlanningWorld,
        avoid_sources: FrozenSet[SlotRef],
        avoid_targets: FrozenSet[SlotRef],
        start: Optional[SlotRef],
    ) -> None:
        self.system = planner.system
        self.sort_map = planner.sort_map
        self.cost = planner.cost
        self.world = world
        self.avoid_sources = avoid_sources
        self.avoid_targets = avoid_targets
        self.previous = start
        self.moves: List[Move] = []
        self.unmet: List[str] = []
        self.awaiting_observation = False  # după o dezvelire, restul planului depinde de ce se va vedea
        self.warnings: List[str] = []
        unobserved = sorted({s.location for s in world.blocked if all(
            t in world.blocked for t in world.grid_slots([s.location]))})
        for name in unobserved:
            self.warnings.append("locația {} nu e observată: nu e folosită".format(name))
        untrusted = [s for s in world.slots() for c in world.stack(s)[-1:] if not c.trusted]
        if untrusted:
            self.warnings.append("{} cub(uri) cu culoare nesigură (sub planning.min_confidence={}) nu sunt mutate: {}".format(
                len(untrusted), self.system.planning.min_confidence, ", ".join(str(s) for s in untrusted[:6])))

    # ------------------------------------------------------------------ utilitare
    def _note(self, bucket: List[str], text: str) -> None:
        if text not in bucket:
            bucket.append(text)

    def _usable_source(self, slot: SlotRef) -> bool:
        return slot not in self.world.blocked and slot not in self.avoid_sources

    def _usable_target(self, slot: SlotRef) -> bool:
        return slot not in self.world.blocked and slot not in self.avoid_targets

    def _cost(self, source: SlotRef, target: SlotRef) -> Tuple[float, bool]:
        return self.cost.move_cost(self.world, self.previous, source, target)

    def _sources(self) -> List[SlotRef]:
        """Locurile de unde se pot lua cuburi: celulele șabloanelor active și punctele tampon."""
        return self.world.grid_slots() + [SlotRef(name) for name in self.system.planning.buffer_points]

    def _staging(self, source: SlotRef, exclude: Set[SlotRef]) -> Optional[SlotRef]:
        """Cea mai ieftină celulă liberă din `staging_locations`; abia apoi un punct tampon cu loc."""
        options = []
        for slot in self.world.grid_slots(self.system.planning.staging_locations):
            if slot in exclude or slot == source or not self.world.is_free_cell(slot) or not self._usable_target(slot):
                continue
            cost, _ = self.cost.move_cost(self.world, source, source, slot)
            options.append((0, cost, _slot_key(slot), slot))
        for name in self.system.planning.buffer_points:
            slot = SlotRef(name)
            if slot in exclude or slot == source or not self._usable_target(slot):
                continue
            if self.world.height(slot) > self.system.planning.buffer_max_level:
                continue
            cost, _ = self.cost.move_cost(self.world, source, source, slot)
            options.append((1, cost, _slot_key(slot), slot))
        return min(options)[3] if options else None

    def _base_uv(self, slot: SlotRef, cube: PlanCube, height: int) -> Optional[Tuple[float, float]]:
        """Centrul bazei cubului din vârful unei stive de `height` cuburi (corectat de paralaxă), dacă e cunoscut."""
        if slot.is_zone or cube.uv_top is None:
            return None
        loc = self.system.location(slot.location)
        return correct_parallax(cube.uv_top, height * self.system.cube.size_mm, loc.camera_height_mm, loc.nadir_uv)

    def _footprints(self, slot: SlotRef, above_level: int) -> List[Footprint]:
        """Bazele vecinilor din același caroiaj care ajung mai sus de nivelul `above_level` (ceilalți sunt sub degete)."""
        loc = self.system.location(slot.location)
        prints = []
        for other in self.world.grid_slots([slot.location]):
            stack = self.world.stack(other)
            if other == slot or len(stack) <= above_level:
                continue
            assert other.cell is not None
            uv = self._base_uv(other, stack[-1], len(stack)) or (other.cell.col + 0.5, other.cell.row + 0.5)
            prints.append(footprint_from_uv(uv, stack[-1].yaw_deg, loc.cell_mm, self.system.cube.size_mm))
        return prints

    def _pick_action(self, slot: SlotRef, cube: PlanCube, level: int) -> PickAction:
        uv = self._base_uv(slot, cube, level + 1)
        yaw = cube.yaw_deg if cube.yaw_deg is not None else 0.0
        gripper = self.system.gripper
        open_mm, status = None, ""
        if gripper is not None:
            if slot.is_zone or slot.cell is None:  # zone/tampoane: fără vecini, dar poziția nu se vede: mai larg
                side = self.system.cube.size_mm
                open_mm = round(min(gripper.max_open_mm, side + 2 * gripper.point_approach_margin_mm), 1)
                return PickAction(slot, level, yaw_deg=float(yaw), uv=None, open_mm=open_mm, grasp="centered")
            else:
                loc = self.system.location(slot.location)
                center = uv or (slot.cell.col + 0.5, slot.cell.row + 0.5)
                target = footprint_from_uv(center, yaw, loc.cell_mm, self.system.cube.size_mm)
                grasp = plan_grasp(target, clearances(target, self._footprints(slot, level)), gripper, loc.cell_mm)
                if uv is not None or not grasp.centered:  # offset-ul (regula B) se trimite prin uv
                    uv = (center[0] + grasp.offset_uv[0], center[1] + grasp.offset_uv[1])
                if not grasp.feasible:
                    self._note(self.warnings, "prindere riscantă pe {}: {}".format(slot, "; ".join(grasp.notes)))
            open_mm, status = grasp.open_mm, grasp.status
        if uv is not None:
            loc = self.system.location(slot.location)
            uv = (round(min(max(uv[0], 0.0), float(loc.cols), 3.0), 4), round(min(max(uv[1], 0.0), float(loc.rows), 3.0), 4))
        return PickAction(slot, level, yaw_deg=float(yaw), uv=uv, open_mm=open_mm, grasp=status)

    def _place_action(self, slot: SlotRef, level: int) -> PlaceAction:
        """Eliberarea cu deschidere mică (regula C), cât permit vecinii slotului țintă. Pe o stivă, cubul se pune
        peste cel de dedesubt (poziția lui văzută), nu în centrul celulei: decalajele nu se adună."""
        uv = None
        if level > 0 and not slot.is_zone:
            below = self.world.stack(slot)[level - 1] if self.world.height(slot) > level else None
            uv = self._base_uv(slot, below, level) if below is not None else None
            if uv is not None:
                loc = self.system.location(slot.location)
                uv = (round(min(max(uv[0], 0.0), float(loc.cols), 3.0), 4), round(min(max(uv[1], 0.0), float(loc.rows), 3.0), 4))
        gripper = self.system.gripper
        if gripper is None:
            return PlaceAction(slot, level, 0.0, uv=uv)
        side = self.system.cube.size_mm
        gaps = Clearances()
        if not slot.is_zone and slot.cell is not None:
            loc = self.system.location(slot.location)
            target = footprint_from_uv(uv or (slot.cell.col + 0.5, slot.cell.row + 0.5), 0.0, loc.cell_mm, side)
            gaps = clearances(target, self._footprints(slot, level))
        release = plan_release(gaps, gripper, side)
        if not release.feasible:
            self._note(self.warnings, "eliberare riscantă pe {}: {}".format(slot, "; ".join(release.notes)))
        return PlaceAction(slot, level, 0.0, release_mm=release.release_mm, uv=uv)

    def _estimate(self, source: SlotRef, target: SlotRef) -> float:
        timing = self.system.timing
        observed = 0
        if self.system.planning.camera_verify == "each_move":
            observed = len({s.location for s in (source, target) if not s.is_zone})
        return timing.pick_s + timing.place_s + observed * timing.observe_s

    def add_move(self, source: SlotRef, target: SlotRef, purpose: str, reason: str) -> None:
        cube = self.world.top(source)
        assert cube is not None, "mutare dintr-un slot gol: {}".format(source)
        level = self.world.height(source) - 1
        place_level = self.world.height(target)
        cost, substitute = self._cost(source, target)
        pick = self._pick_action(source, cube, level)
        self.world.move_top(source, target)
        # Eliberarea se calculează după mutare: cubul ținut nu e propriul vecin la destinație.
        place = self._place_action(target, place_level)
        self.moves.append(Move(
            cube=CubeRef(source, level, cube.color, cube.top_face),
            pick=pick,
            place=place,
            purpose=purpose,
            reason=reason,
            cost_deg=cost,
            cost_is_substitute=substitute,
            estimated_s=self._estimate(source, target),
        ))
        self.previous = target

    def _highest(self, slot: SlotRef, color: Color, face: Optional[TopFace] = None) -> Optional[Tuple[int, int]]:
        """(indexul celui mai înalt cub de încredere de culoarea dată, nr. de cuburi deasupra) sau None.
        Un cub sub un cub nesigur nu poate fi dezgropat (cubul nesigur nu se mută)."""
        stack = self.world.stack(slot)
        for j in range(len(stack) - 1, -1, -1):
            cube = stack[j]
            if cube.trusted and cube.color is color and (face is None or cube.top_face is face):
                blockers = stack[j + 1:]
                return (j, len(blockers)) if all(b.trusted for b in blockers) else None
        return None

    def _hidden_count(self) -> int:
        return sum(1 for slot in self._sources() for cube in self.world.stack(slot)[:-1] if _is_hidden(cube))

    def _reveal(self, exclude: Set[SlotRef], why: str) -> bool:
        """Dezvelește cel mai ieftin cub ascuns (nivel dedus din înălțime, culoare necunoscută): mută cubul de deasupra."""
        options = []
        for slot in self._sources():
            stack = self.world.stack(slot)
            if slot in exclude or not self._usable_source(slot) or len(stack) < 2:
                continue
            if any(_is_hidden(c) for c in stack[:-1]) and stack[-1].trusted:
                cost, _ = self._cost(slot, slot)
                options.append((cost, _slot_key(slot), slot))
        if not options:
            return False
        slot = min(options)[2]
        if not self._clear_top(slot, exclude | {slot}, "dezvelește cubul ascuns de pe {} ({})".format(slot, why)):
            return False
        self.awaiting_observation = True
        self._note(self.warnings, "planul continuă după reobservare (cubul dezvelit are culoarea încă necunoscută)")
        return True

    def _deficit_note(self, color: Color, missing: int) -> None:
        hidden = self._hidden_count()
        if hidden >= missing:
            self._note(self.warnings, "{} la vedere; caut sub {} cub(uri) ascunse".format(missing_text(color, missing), hidden))
        else:
            self._note(self.unmet, missing_text(color, missing))

    def result(self) -> Plan:
        return Plan(
            moves=tuple(self.moves),
            complete=not self.unmet and not self.awaiting_observation,
            unmet=tuple(self.unmet),
            warnings=tuple(self.warnings),
            estimated_s=round(sum(m.estimated_s for m in self.moves), 2),
        )

    # ------------------------------------------------------------------ S2: turn
    def tower(self, goal: TowerGoal) -> None:
        limit = self.system.planning.max_tower_level + 1
        order = goal.order
        if len(order) > limit:
            self._note(self.unmet, "nivel peste limită: turnul cere {} cuburi, maximum {} (planning.max_tower_level={})".format(
                len(order), limit, self.system.planning.max_tower_level))
            order = order[:limit]
        target = goal.target
        if target not in self.world.stacks or target.is_zone:
            self._note(self.unmet, "celula țintă {} nu există în configurație".format(target))
            return
        if not self._usable_target(target):
            self._note(self.unmet, "celula țintă {} nu e observată sigur sau e inaccesibilă".format(target))
            return
        self._tower_deficits(order, target)
        for _ in range(MAX_STEPS):
            stack = self.world.stack(target)
            k = 0
            while k < len(stack) and k < len(order) and stack[k].trusted and stack[k].color is order[k]:
                k += 1
            if k < len(stack):  # deasupra prefixului corect sunt cuburi greșite: pleacă, de sus în jos
                top = stack[-1]
                if not top.trusted:
                    self._note(self.unmet, "pe celula țintă e un cub nesigur ({}); nu îl mut".format(top.color.value))
                    return
                staging = self._staging(target, {target})
                if staging is None:
                    self._note(self.unmet, "nu există celule libere pentru a elibera celula țintă")
                    return
                self.add_move(target, staging, "clear", "eliberează celula țintă (cubul {} nu e la locul lui)".format(
                    color_name(top.color)))
                continue
            if k == len(order):
                return
            color = order[k]
            best = None
            for slot in self._sources():
                if slot == target or not self._usable_source(slot):
                    continue
                found = self._highest(slot, color)
                if found is None:
                    continue
                cost, _ = self._cost(slot, target)
                key = (found[1], cost, _slot_key(slot))
                if best is None or key < best[0]:
                    best = (key, slot)
            if best is None:
                if self._reveal({target}, "caut un cub {}".format(color_name(color))):
                    return
                self._note(self.unmet, "nivelul {} ({}) nu poate fi construit: niciun cub accesibil".format(k, color_name(color)))
                return
            (blockers, _, _), slot = best
            if blockers:
                if not self._clear_top(slot, {target, slot}, "dezgroapă cubul {} de sub {} cub(uri)".format(
                        color_name(color), blockers)):
                    return
                continue
            self.add_move(slot, target, "goal", "nivelul {} al turnului: {}".format(k, color_name(color)))
        self._note(self.unmet, "planul a depășit {} de mutări".format(MAX_STEPS))

    def _tower_deficits(self, order: Tuple[Color, ...], target: SlotRef) -> None:
        total = sum(1 for s in self._sources() for c in self.world.stack(s) if c.trusted)
        if total < len(order):
            self._note(self.unmet, "prea puține cuburi: turnul cere {}, pe masă sunt {} de încredere".format(len(order), total))
        for color in KNOWN_COLORS:
            need = sum(1 for c in order if c is color)
            have = self.world.count_trusted(color)
            if need > have:
                self._deficit_note(color, need - have)

    def _clear_top(self, slot: SlotRef, exclude: Set[SlotRef], reason: str) -> bool:
        staging = self._staging(slot, exclude)
        if staging is None:
            self._note(self.unmet, "nu există celule libere pentru mutări intermediare")
            return False
        self.add_move(slot, staging, "clear", reason)
        return True

    # ------------------------------------------------------------------ S1: reproducere
    def reproduce(self, goal: ReproduceGoal) -> None:
        if goal.location not in self.world.grid_sizes:
            self._note(self.unmet, "locația țintă {} nu există în configurație".format(goal.location))
            return
        targets = {SlotRef(goal.location, coord): t for coord, t in goal.cells.items()}
        policy = self.system.planning.match_top_face
        bottoms = ["{} {}".format(slot, color_name(targets[slot].bottom or Color.UNKNOWN))
                   for slot in sorted(targets, key=_slot_key) if targets[slot].bottom is not None]
        if bottoms:
            self._note(self.warnings, "fața de jos cerută pentru {} celule: neverificabil (camera vede doar fața de sus): {}".format(
                len(bottoms), ", ".join(bottoms)))
        need: Dict[Color, int] = {}
        for t in targets.values():
            if t.top is not None:
                need[t.top] = need.get(t.top, 0) + 1
        for color, count in need.items():
            have = self.world.count_trusted(color)
            if count > have:
                self._deficit_note(color, count - have)

        def required_face(t) -> Optional[TopFace]:
            return t.face if policy == "require" and t.face is not TopFace.UNKNOWN else None

        def satisfied(slot: SlotRef) -> bool:
            t = targets[slot]
            stack = self.world.stack(slot)
            if t.top is None:
                return not stack
            face = required_face(t)
            return (len(stack) == 1 and stack[0].trusted and stack[0].color is t.top
                    and (face is None or stack[0].top_face is face))

        def wanting(color: Color, face: TopFace) -> Optional[SlotRef]:
            """O celulă țintă goală, nesatisfăcută, care cere exact acest cub (pentru mutări directe)."""
            options = []
            for slot, t in targets.items():
                if t.top is color and not self.world.stack(slot) and self._usable_target(slot):
                    rf = required_face(t)
                    if rf is None or rf is face:
                        options.append((_slot_key(slot), slot))
            return min(options)[1] if options else None

        for _ in range(MAX_STEPS):
            unsatisfied = [s for s in sorted(targets, key=_slot_key) if not satisfied(s)]
            if not unsatisfied:
                return
            locked = {s for s in targets if satisfied(s)}
            for slot in unsatisfied:
                if not self._usable_target(slot):
                    self._note(self.unmet, "{}: celulă neobservată sigur sau inaccesibilă".format(slot))
            workable = [s for s in unsatisfied if self._usable_target(s)]
            # 1) umple o celulă goală cu cubul potrivit (cel mai ieftin; la „prefer”, întâi fața potrivită)
            best = None
            for slot in workable:
                t = targets[slot]
                if t.top is None or self.world.stack(slot):
                    continue
                for source in self._sources():
                    if source == slot or source in locked or not self._usable_source(source):
                        continue
                    found = self._highest(source, t.top, required_face(t))
                    if found is None:
                        continue
                    cube = self.world.stack(source)[found[0]]
                    mismatch = int(policy == "prefer" and t.face is not TopFace.UNKNOWN and cube.top_face is not t.face)
                    cost, _ = self._cost(source, slot)
                    key = (mismatch, found[1], cost, _slot_key(slot), _slot_key(source))
                    if best is None or key < best[0]:
                        best = (key, source, slot)
            if best is not None:
                (mismatch, blockers, _, _, _), source, slot = best
                if blockers:
                    top = self.world.top(source)
                    assert top is not None
                    direct = wanting(top.color, top.top_face)
                    if direct is not None and direct != slot:
                        self.add_move(source, direct, "goal", "{} pe {} (dezgroapă alt cub)".format(color_name(top.color), direct))
                    elif not self._clear_top(source, set(targets) | {source}, "dezgroapă cubul {} pentru {}".format(
                            color_name(targets[slot].top or top.color), slot)):
                        return
                    continue
                note = " (fața de sus diferă)" if mismatch else ""
                self.add_move(source, slot, "goal", "{} pe {}{}".format(color_name(targets[slot].top or Color.UNKNOWN), slot, note))
                continue
            # 2) golește o celulă țintă ocupată greșit
            clear = None
            for slot in workable:
                top = self.world.top(slot)
                if top is None:
                    continue
                if not top.trusted:
                    self._note(self.unmet, "{}: cub nesigur ({}) pe o celulă țintă; nu îl mut".format(slot, top.color.value))
                    continue
                direct = wanting(top.color, top.top_face)
                destination = direct if direct is not None and direct != slot else self._staging(slot, set(targets) | {slot})
                if destination is None:
                    continue
                cost, _ = self._cost(slot, destination)
                key = (cost, _slot_key(slot))
                if clear is None or key < clear[0]:
                    clear = (key, slot, destination, direct is not None and destination == direct)
            if clear is not None:
                _, slot, destination, is_goal = clear
                top = self.world.top(slot)
                assert top is not None
                if is_goal:
                    self.add_move(slot, destination, "goal", "{} pe {}".format(color_name(top.color), destination))
                else:
                    self.add_move(slot, destination, "clear", "golește {} (cubul {} nu e cerut acolo)".format(slot, color_name(top.color)))
                continue
            if self._reveal(set(targets), "caut cuburi pentru aranjament"):
                return
            for slot in workable:
                t = targets[slot]
                if t.top is None:
                    self._note(self.unmet, "{}: trebuie golită, dar nu există unde muta cubul".format(slot))
                elif self.world.stack(slot):
                    self._note(self.unmet, "{}: ocupată greșit și nu poate fi golită (nicio celulă liberă și niciun punct "
                                           "tampon: planning.buffer_points)".format(slot))
                else:
                    self._note(self.unmet, "{}: niciun cub {} accesibil".format(slot, color_name(t.top)))
            return
        self._note(self.unmet, "planul a depășit {} de mutări".format(MAX_STEPS))

    # ------------------------------------------------------------------ S3: sortare
    def sort(self) -> None:
        max_level = self.sort_map.max_level
        unknown_policy = self.sort_map.unknown_color_policy
        for _ in range(MAX_STEPS):
            options = []
            for slot in self._sources():
                top = self.world.top(slot)
                if top is None or not self._usable_source(slot):
                    continue
                if not top.trusted:
                    buried = [c for c in self.world.stack(slot)[:-1] if c.trusted]
                    if unknown_policy == "staging" and top.color is Color.UNKNOWN and buried:
                        staging = self._staging(slot, {slot})
                        if staging is not None:
                            cost, _ = self._cost(slot, staging)
                            options.append((cost, _slot_key(slot), slot, staging, "staging"))
                    continue
                zone = self.sort_map.zone_for(top.color)
                if zone is None:
                    continue
                zslot = SlotRef(zone.name)
                if zslot not in self.world.stacks or not self._usable_target(zslot):
                    continue
                if self.world.height(zslot) > max_level:
                    continue
                cost, _ = self._cost(slot, zslot)
                options.append((cost, _slot_key(slot), slot, zslot, "sort"))
            if not options:
                break
            _, _, source, target, purpose = min(options, key=lambda o: (o[0], o[1]))
            top = self.world.top(source)
            assert top is not None
            if purpose == "sort":
                self.add_move(source, target, "sort", "{} pe {}".format(color_name(top.color), target))
            else:
                self.add_move(source, target, "staging", "cub de culoare necunoscută mutat de pe cuburi sortabile")
        self._sort_leftovers(max_level)

    def _sort_leftovers(self, max_level: int) -> None:
        counts: Dict[str, int] = {}
        for slot in self._sources():
            stack = self.world.stack(slot)
            for index, cube in enumerate(stack):
                if not cube.trusted:
                    continue
                zone = self.sort_map.zone_for(cube.color)
                if zone is None:
                    reason = "nicio zonă pentru culoarea {}".format(color_name(cube.color))
                elif SlotRef(zone.name) in self.avoid_targets:
                    reason = "zona {} e inaccesibilă".format(zone.name)
                elif self.world.height(SlotRef(zone.name)) > max_level:
                    reason = "nivel peste limită: zona {} e plină (sort_map.max_level={})".format(zone.name, max_level)
                elif slot in self.world.blocked:
                    reason = "cub pe o celulă neobservată sigur"
                elif slot in self.avoid_sources:
                    reason = "cub inaccesibil (prindere eșuată de prea multe ori)"
                elif any(not c.trusted for c in stack[index + 1:]):
                    reason = "cub blocat sub un cub nesigur (unknown_color_policy={})".format(self.sort_map.unknown_color_policy)
                else:
                    reason = "cub nesortat"
                counts[reason] = counts.get(reason, 0) + 1
        for reason, count in counts.items():
            self._note(self.unmet, "{} ({} cub(uri))".format(reason, count))
        unknown = [s for s in self.world.grid_slots() for c in self.world.stack(s) if c.color is Color.UNKNOWN]
        if unknown and self.sort_map.unknown_color_policy == "leave":
            self._note(self.warnings, "{} cub(uri) de culoare necunoscută rămân pe loc (unknown_color_policy=leave)".format(len(unknown)))


def _is_hidden(cube: PlanCube) -> bool:
    """Un cub dedus doar din înălțimea stivei: culoarea nu a fost văzută niciodată."""
    return cube.color is Color.UNKNOWN and cube.confidence == 0.0
