import unittest
from datetime import datetime, timedelta

import jwt

from pyniryo.nate._internal.utils import get_token_expiration


def _make_jwt(exp: datetime | None = None, include_exp: bool = True) -> str:
    """Helper to create a JWT token for testing."""
    payload = {}
    if include_exp:
        payload['exp'] = int(exp.timestamp()) if exp else int((datetime.now() + timedelta(hours=1)).timestamp())
    return jwt.encode(payload, key='very_long_secret_so_there_is_no_warning', algorithm='HS256')


class TestGetTokenExpiration(unittest.TestCase):

    def test_valid_token_returns_correct_datetime(self):
        """Test that a valid JWT with exp claim returns the correct expiration datetime."""
        exp = datetime.now() + timedelta(hours=2)
        token = _make_jwt(exp=exp)

        result = get_token_expiration(token)

        # Allow 1 second tolerance due to timestamp integer truncation
        self.assertAlmostEqual(result.timestamp(), exp.timestamp(), delta=1)

    def test_token_without_exp_raises_value_error(self):
        """Test that a JWT token without an 'exp' claim raises ValueError."""
        token = _make_jwt(include_exp=False)

        with self.assertRaises(ValueError) as context:
            get_token_expiration(token)

        self.assertIn('expiration claim', str(context.exception))

    def test_invalid_token_raises_value_error(self):
        """Test that an invalid/malformed token raises ValueError."""
        with self.assertRaises(ValueError) as context:
            get_token_expiration('this.is.not.a.valid.jwt')

        self.assertIn('Invalid token', str(context.exception))

    def test_empty_string_raises_value_error(self):
        """Test that an empty string raises ValueError."""
        with self.assertRaises(ValueError):
            get_token_expiration('')

    def test_past_expiry_token_returns_past_datetime(self):
        """Test that a token with a past exp claim returns a past datetime."""
        exp = datetime.now() - timedelta(hours=1)
        token = _make_jwt(exp=exp)

        result = get_token_expiration(token)

        self.assertAlmostEqual(result.timestamp(), exp.timestamp(), delta=1)
        self.assertLess(result, datetime.now())

    def test_return_type_is_datetime(self):
        """Test that the return type is always a datetime object."""
        token = _make_jwt()

        result = get_token_expiration(token)

        self.assertIsInstance(result, datetime)


if __name__ == '__main__':
    unittest.main()


class TestEnsureUtc(unittest.TestCase):

    def test_naive_datetime_is_rejected(self):
        from pyniryo.nate._internal.utils import ensure_utc
        from pyniryo.nate.exceptions import DataValidationError

        with self.assertRaises(DataValidationError):
            ensure_utc(datetime(2026, 1, 1, 12, 0, 0))

    def test_aware_datetime_is_converted_to_utc(self):
        from datetime import timezone
        from pyniryo.nate._internal.utils import ensure_utc

        paris = timezone(timedelta(hours=2))
        result = ensure_utc(datetime(2026, 1, 1, 12, 0, 0, tzinfo=paris))

        self.assertEqual(result, datetime(2026, 1, 1, 10, 0, 0, tzinfo=timezone.utc))
        self.assertEqual(result.tzinfo, timezone.utc)
