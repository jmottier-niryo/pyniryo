import os
from datetime import datetime, timezone
from typing import TypeVar
import jwt

from ..exceptions import DataValidationError

_valid_types = str | int | float | bool
_T = TypeVar("_T", bound=str | int | float | bool)


def fetch_from_env(key: str, _type: type[_T]) -> _T | None:
    """
    Fetch a value from environment variables and convert it to the specified type.

    :param key: The environment variable key.
    :param _type: The target type to convert to.
    :return: The converted value or the default.
    :raises ValueError: If the conversion fails.
    :raises TypeError: If the type is not supported.
    """
    value = os.getenv(key)

    if value is None:
        return None

    if _type is bool:
        return _type(value.lower() in ['true', '1', 'yes'])
    elif _type in [int, float, str]:
        try:
            return _type(value)
        except ValueError as e:
            raise ValueError(f'Error converting environment variable {key} to type {_type}({value})') from e
    else:
        raise TypeError(f'Unsupported type {_type}. Must be one of {_valid_types}')


def get_token_expiration(token: str) -> datetime:
    """
    Extract the expiration date from a JWT token.

    :param token: The JWT token string.
    :return: The expiration date as a datetime object.
    :raises ValueError: If the token is invalid or does not contain an expiration claim.
    """

    try:
        decoded_token = jwt.decode(token, options={"verify_signature": False})
        exp_timestamp = decoded_token.get('exp')
        if exp_timestamp is None:
            raise ValueError("Token does not contain an expiration claim.")
        return datetime.fromtimestamp(exp_timestamp)
    except Exception as e:
        raise ValueError(f"Invalid token: {e}") from e


def ensure_utc(dt: datetime) -> datetime:
    """
    Convert a datetime to UTC for the wire, which carries RFC 3339 timestamps and reads a naive one as an error.

    :param dt: A timezone-aware datetime.
    :return: The same instant in UTC.
    :raises DataValidationError: If the datetime carries no timezone.
    """
    if dt.tzinfo is None or dt.tzinfo.utcoffset(dt) is None:
        raise DataValidationError(f'datetime {dt.isoformat()} carries no timezone, give an aware one')
    return dt.astimezone(timezone.utc)
