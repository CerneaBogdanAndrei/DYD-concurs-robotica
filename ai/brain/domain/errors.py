"""Ierarhia de excepții a subsistemului Brain."""

from __future__ import annotations

from typing import Any


class BrainError(Exception):
    """Baza tuturor erorilor subsistemului."""


class DomainValidationError(BrainError, ValueError):
    """Valoare de domeniu invalidă (coordonată, încredere, unghi etc.)."""


class ConfigError(BrainError):
    """Fișier de configurare invalid. Mesajul identifică fișierul, cheia și valoarea."""

    def __init__(self, file: str, key: str, value: Any, message: str) -> None:
        self.file = file
        self.key = key
        self.value = value
        self.message = message
        super().__init__("{}: cheia '{}': {} (valoare: {!r})".format(file, key, message, value))


class PerceptionInputError(BrainError, ValueError):
    """Date de intrare invalide pentru pipeline-ul de viziune (cadre lipsă, formă greșită)."""


class SceneStateError(BrainError):
    """Baza erorilor de memorie persistentă."""


class SceneStateNotFound(SceneStateError):
    """Fișierul de stare nu există."""


class SceneStateCorrupt(SceneStateError):
    """Fișierul de stare nu poate fi citit sau nu respectă schema."""


class SceneStateIncompatible(SceneStateError):
    """Fișierul de stare are o versiune de schemă diferită de cea curentă."""


class SceneStateExists(SceneStateError):
    """Salvarea ar suprascrie un fișier existent fără permisiune explicită."""


class ArmError(BrainError):
    """Baza erorilor de mișcare. După o astfel de eroare starea fizică poate fi necunoscută."""


class InvalidCommand(ArmError, ValueError):
    """Comanda e respinsă înainte de trimitere (argument greșit, în afara limitelor). Brațul nu s-a mișcat."""


class CalibrationMissing(ArmError):
    """Poza sau locul cerut nu e calibrat în serviciul de mișcare (codul `calibrare`)."""


class ArmCommunicationError(ArmError):
    """Serviciul/hardware-ul nu răspunde sau a raportat o eroare (I2C). Starea fizică e NECUNOSCUTĂ."""


class ArmTimeout(ArmCommunicationError):
    """Comanda nu s-a terminat în timpul permis. Starea fizică e NECUNOSCUTĂ."""


class CapabilityUnavailable(ArmError):
    """Adaptorul nu oferă funcția cerută. Nu se raportează niciodată un succes fals."""


class SafetyRejected(ArmError):
    """Operația a fost oprită de verificările de siguranță înainte de a fi trimisă."""
