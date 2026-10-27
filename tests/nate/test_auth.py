from datetime import datetime, timedelta, timezone

from unittest.mock import MagicMock, call
from uuid import uuid4

from pyniryo.nate._internal import paths_gen, transport_models, topics_gen
from pyniryo.nate.models import auth
from pyniryo.nate.components.auth import Auth

from .base import BaseTestComponent, mock_format


class TestAuth(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.auth = Auth(http_client=self.http_client, mqtt_client=self.mqtt_client, correlation_id=self.correlation_id)

    def tearDown(self):
        del self.auth

    def test_login(self):
        expire_date = datetime.now(timezone.utc) + timedelta(days=1)
        self.http_client.post.return_value = transport_models.s.Token(
            expires_at=expire_date,
            id=uuid4(),
            created_at=datetime.now(),
            token='generated_token',
        )

        token = self.auth.login(login='mail@mail.com', password='password', expires_at=expire_date)
        self.http_client.post.assert_called_once()
        endpoint, response_model, login_model = self.http_client.post.call_args[0]
        self.assertEqual(endpoint, paths_gen.Authentication.LOGIN)
        self.assertIsInstance(login_model, transport_models.s.Login)
        self.assertEqual(login_model.login, 'mail@mail.com')
        self.assertEqual(login_model.password, 'password')
        self.assertEqual(login_model.expires_at, expire_date)
        self.assertEqual(response_model, transport_models.s.Token)
        self.assertEqual(token.token, 'generated_token')

    def test_on_user_logged_in_specific_user(self):
        """Test subscribing to login events for a specific user."""
        user_id = str(uuid4())
        user_callback = MagicMock(spec=lambda u_id, u_logged_in: None)

        self.auth.on_user_logged_in(user_callback, user_id)

        self.mqtt_client.subscribe.assert_called_once()
        topic = self.mqtt_client.format(topics_gen.Users.USER_LOGGED_IN, user_id=user_id)
        self.assertEqual(self.mqtt_client.subscribe.call_args[0][0], topic)

        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        internal_callback(topic, transport_models.a.UserEvent())
        user_callback.assert_called_once_with(user_id, auth.UserEvent())

    def test_on_user_logged_in_any_user(self):
        """Test subscribing to login events for any user."""

        user_callback = MagicMock(spec=lambda u_id, u_logged_in: None)
        self.auth.on_user_logged_in(user_callback)

        self.mqtt_client.subscribe.assert_called_once()
        self.assertEqual(self.mqtt_client.subscribe.call_args[0][0],
                         self.mqtt_client.format(topics_gen.Users.USER_LOGGED_IN, user_id='+'))

        expected_calls = []
        for _ in range(3):
            user_id = str(uuid4())
            self.auth.on_user_logged_in(user_callback, user_id)

            topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
            internal_callback(topic, transport_models.a.UserEvent())
            expected_calls.append(call(user_id, auth.UserEvent()))

        user_callback.assert_has_calls(expected_calls)

    def test_on_user_logged_out_specific_user(self):
        """Test subscribing to logout events for a specific user."""
        user_id = str(uuid4())
        user_callback = MagicMock(spec=lambda u_id, u_logged_out: None)

        self.auth.on_user_logged_out(user_callback, user_id)

        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]

        internal_callback(topic, transport_models.a.UserEvent())
        user_callback.assert_called_once_with(user_id, auth.UserEvent())

    def test_on_user_logged_out_any_user(self):
        """Test subscribing to logout events for any user."""

        user_callback = MagicMock(spec=lambda u_id, u_logged_out: None)
        self.auth.on_user_logged_out(user_callback)

        self.mqtt_client.subscribe.assert_called_once()
        self.assertEqual(self.mqtt_client.subscribe.call_args[0][0],
                         self.mqtt_client.format(topics_gen.Users.USER_LOGGED_OUT, user_id='+'))

        expected_calls = []
        for _ in range(3):
            user_id = str(uuid4())
            self.auth.on_user_logged_out(user_callback, user_id)

            topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
            internal_callback(topic, transport_models.a.UserEvent())
            expected_calls.append(call(user_id, auth.UserEvent()))

        user_callback.assert_has_calls(expected_calls)

    def test_login_rejects_a_naive_expiry(self):
        from pyniryo.nate.exceptions import DataValidationError

        with self.assertRaises(DataValidationError):
            self.auth.login('mail@mail.com', 'password', datetime.now() + timedelta(days=1))
        self.http_client.post.assert_not_called()

    def test_on_user_logged_in_any_user_reads_the_id_from_the_topic(self):
        self.mqtt_client.format.side_effect = mock_format
        user_callback = MagicMock(spec=lambda u_id, u_logged_in: None)
        self.auth.on_user_logged_in(user_callback)
        topic, internal_callback, _ = self.mqtt_client.subscribe.call_args[0]

        internal_callback(topic.replace('+', 'user-42'), transport_models.a.UserEvent())

        user_callback.assert_called_once_with('user-42', auth.UserEvent())
