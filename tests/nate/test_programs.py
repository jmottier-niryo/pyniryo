# type: ignore

import unittest
from datetime import datetime
from io import BytesIO
from unittest.mock import MagicMock, patch
from uuid import uuid4

from pyniryo.nate.models import programs, ExecutorStatus
from pyniryo.nate.models.programs import ProgramExecutorStatus
from pyniryo.nate._internal import transport_models, paths_gen, topics_gen
from pyniryo.nate.components.programs import Programs, ExecutionBaseCommand
from pyniryo.nate.exceptions import PyNiryoError

from .base import BaseTestComponent

base_program = programs.Program(
    id=str(uuid4()),
    name='test_program',
    type=programs.ProgramType.PYTHON312,
)

base_execution = programs.ProgramExecution(
    id=str(uuid4()),
    program_id=base_program.id,
    context=programs.ProgramExecutionContext(environment={}, arguments=[]),
    started_at=datetime.now(),
    finished_at=datetime.now(),
    exit_code=0,
    status=ExecutorStatus.RUNNING,
)


class TestPrograms(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.programs = Programs(http_client=self.http_client,
                                 mqtt_client=self.mqtt_client,
                                 correlation_id=self.correlation_id)

    def tearDown(self):
        del self.programs

    def test_get_all(self):
        """Test getting all programs."""
        t_models = transport_models.ProgramList([base_program.to_transport_model()])
        self.http_client.get.return_value = t_models

        all_programs = self.programs.get_all()

        self.http_client.get.assert_called_once_with(paths_gen.Programs.GET_ALL_PROGRAMS, transport_models.ProgramList)
        self.assertEqual(len(all_programs), 1)
        self.assertEqual(all_programs[0].id, base_program.id)
        self.assertEqual(all_programs[0].name, base_program.name)

    def test_create(self):
        """Test creating a program with file upload."""
        self.http_client.post.return_value = base_program.to_transport_model()

        file_content = b"print('Hello, World!')"
        file = BytesIO(file_content)

        program = self.programs.create('test_program', file, programs.ProgramType.PYTHON312)

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args
        self.assertEqual(call_args[0][0], paths_gen.Programs.CREATE_PROGRAM)
        self.assertEqual(call_args[0][1], transport_models.s.Program)
        self.assertIn('files', call_args[1])
        self.assertEqual(program.id, base_program.id)

    def test_create_leaves_the_file_field_out_of_the_form(self):
        self.http_client.post.return_value = base_program.to_transport_model()

        self.programs.create('test_program', BytesIO(b"x"), programs.ProgramType.PYTHON312)

        self.assertEqual(self.http_client.post.call_args.kwargs['exclude'], {'file'})

    def test_create_without_a_name(self):
        self.http_client.post.return_value = base_program.to_transport_model()

        self.programs.create(None, BytesIO(b"x"), programs.ProgramType.PYTHON312)

        request = self.http_client.post.call_args[0][2]
        self.assertIsNone(request.name)

    def test_get_without_download(self):
        """Test getting a program without downloading content."""
        self.http_client.get.return_value = base_program.to_transport_model()

        program = self.programs.get(base_program.id)

        self.http_client.get.assert_called_once_with(
            paths_gen.Programs.GET_PROGRAM.format(program_id=base_program.id),
            transport_models.s.Program,
        )
        self.http_client.download.assert_not_called()
        self.assertEqual(program.id, base_program.id)

    def test_get_with_download(self):
        """Test getting a program with downloading content."""
        self.http_client.get.return_value = base_program.to_transport_model()

        dst = BytesIO()
        program = self.programs.get(base_program.id, dst)

        self.http_client.get.assert_called_once()
        self.http_client.download.assert_called_once_with(
            paths_gen.Programs.GET_PROGRAM_FILE.format(program_id=base_program.id), dst)
        self.assertEqual(program.id, base_program.id)

    def test_delete(self):
        """Test deleting a program."""
        self.http_client.delete.return_value = None

        self.programs.delete(base_program.id)

        self.http_client.delete.assert_called_once_with(
            paths_gen.Programs.DELETE_PROGRAM.format(program_id=base_program.id),
            transport_models.EmptyPayload,
        )

    def test_update_without_file(self):
        """Test updating program metadata without file upload."""
        updated_program = programs.Program(
            id=base_program.id,
            name='updated_program',
            type=programs.ProgramType.PYTHON312,
        )
        self.http_client.patch.return_value = updated_program.to_transport_model()

        program = self.programs.update(updated_program)

        self.http_client.patch.assert_called_once_with(
            paths_gen.Programs.UPDATE_PROGRAM.format(program_id=updated_program.id),
            transport_models.s.Program,
            updated_program.to_transport_model(),
        )
        self.assertEqual(program.name, 'updated_program')

    def test_update_with_file(self):
        """Test updating program with file upload."""
        self.http_client.patch.return_value = base_program.to_transport_model()

        file_content = b"print('Updated code')"
        src = BytesIO(file_content)

        _ = self.programs.update(base_program, src)

        # Should call patch twice: once for metadata, once for file
        self.assertEqual(self.http_client.patch.call_count, 2)

        # Check file upload call
        file_upload_call = self.http_client.patch.call_args_list[1]
        self.assertEqual(file_upload_call[0][0],
                         paths_gen.Programs.UPLOAD_PROGRAM_FILE.format(program_id=base_program.id))
        self.assertIn('files', file_upload_call[1])

    def test_get_executions(self):
        """Test getting all executions of a program."""
        t_models = transport_models.ProgramExecutionList([base_execution.to_transport_model()])
        self.http_client.get.return_value = t_models

        executions = self.programs.get_executions(base_program.id)

        self.http_client.get.assert_called_once_with(
            paths_gen.Programs.GET_PROGRAM_EXECUTIONS.format(program_id=base_program.id),
            transport_models.ProgramExecutionList,
        )
        self.assertEqual(len(executions), 1)
        self.assertEqual(executions[0].id, base_execution.id)

    def test_get_execution(self):
        """Test getting a specific program execution."""
        self.http_client.get.return_value = base_execution.to_transport_model()

        execution = self.programs.get_execution(base_program.id, base_execution.id)

        self.http_client.get.assert_called_once_with(
            paths_gen.Programs.GET_PROGRAM_EXECUTION.format(program_id=base_program.id, execution_id=base_execution.id),
            transport_models.s.ProgramExecution,
        )
        self.assertEqual(execution.id, base_execution.id)

    def test_execute_without_callbacks(self):
        """Test executing a program without callbacks."""
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=uuid4())

        cmd = self.programs.execute(base_program.id)

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Programs.EXECUTE_PROGRAM.format(program_id=base_program.id))
        self.assertEqual(call_args[1], transport_models.s.FeedbackResponse)

        request = call_args[2]
        self.assertIsInstance(request, transport_models.s.ExecuteProgram)

        self.assertIsInstance(cmd, ExecutionBaseCommand)
        self.assertEqual(cmd.program_id, base_program.id)

    def test_execute_with_environment_and_arguments(self):
        """Test executing a program with environment variables and arguments."""
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=uuid4())

        env = {'KEY': 'value'}
        args = ['arg1', 'arg2']

        _ = self.programs.execute(base_program.id, environment=env, arguments=args)

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(request.context.environment, env)
        self.assertEqual(request.context.arguments, args)

    def test_execute_with_callbacks(self):
        """Test executing a program with output and status callbacks."""
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=uuid4())

        output_callback = MagicMock()
        status_callback = MagicMock()

        cmd = self.programs.execute(base_program.id, on_output=output_callback, on_status=status_callback)

        # Verify MQTT subscriptions were made (one for output, one for status)
        self.assertEqual(self.mqtt_client.subscribe.call_count, 2)

        # Check output subscription
        output_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_OUTPUT,
                                               program_id=base_program.id,
                                               execution_id=cmd.execution_id)
        status_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_STATUS,
                                               program_id=base_program.id,
                                               execution_id=cmd.execution_id)

        subscribed_topics = [call[0][0] for call in self.mqtt_client.subscribe.call_args_list]
        self.assertIn(output_topic, subscribed_topics)
        self.assertIn(status_topic, subscribed_topics)

    def test_execute_refused_cleans_up_the_subscriptions(self):
        from pyniryo.nate.exceptions import ClientError
        unsubscribe_output, unsubscribe_status = MagicMock(), MagicMock()
        self.mqtt_client.subscribe.side_effect = [unsubscribe_output, unsubscribe_status]
        self.http_client.post.side_effect = ClientError(409, '{"error": "held", "code": "robot_held"}')

        with self.assertRaises(ClientError):
            self.programs.execute(base_program.id, on_output=MagicMock())

        unsubscribe_output.assert_called_once()
        unsubscribe_status.assert_called_once()

    def test_execute_omits_an_empty_context(self):
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=uuid4())

        self.programs.execute(base_program.id)

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(request.context.model_dump(exclude_none=True), {})

    def test_execute_without_callbacks_subscribes_only_status(self):
        """Test executing a program without callbacks subscribes only to status topic."""
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=uuid4())

        cmd = self.programs.execute(base_program.id)

        # Only status subscription (output subscription skipped when no on_output callback)
        self.assertEqual(self.mqtt_client.subscribe.call_count, 1)

    def test_executor_status(self):
        """Test getting the executor status."""
        self.http_client.get.return_value = transport_models.s.ProgramsExecutorStatus(
            status=transport_models.s.ExecutorStatus.RUNNING,
        )

        status = self.programs.executor_status()

        self.http_client.get.assert_called_once_with(paths_gen.Programs.GET_PROGRAMS_EXECUTOR_STATUS,
                                                     transport_models.s.ProgramsExecutorStatus)
        self.assertIsInstance(status, ProgramExecutorStatus)
        self.assertEqual(status.status, ExecutorStatus.RUNNING)

    def test_executor_status_at_rest(self):
        """Test getting the executor status of a robot that has run nothing since it started."""
        # Built through model_validate, like the HTTP client does: the raw body is where pydantic
        # rejects a status the generated enum does not carry.
        self.http_client.get.return_value = transport_models.s.ProgramsExecutorStatus.model_validate(
            {'status': 'idle'})

        status = self.programs.executor_status()

        self.assertEqual(status.status, ExecutorStatus.IDLE)
        self.assertIsNone(status.execution)

    def test_pause(self):
        """Test pausing program execution."""
        self.http_client.patch.return_value = None

        self.programs.pause()

        self.http_client.patch.assert_called_once()
        call_args = self.http_client.patch.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Programs.UPDATE_PROGRAMS_EXECUTOR_STATUS)
        request = call_args[2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.PAUSED)

    def test_stop(self):
        """Test stopping program execution without sigkill."""
        self.http_client.patch.return_value = None

        self.programs.stop(sigkill=False)

        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.STOPPED)
        self.assertFalse(request.sigkill)

    def test_stop_with_sigkill(self):
        """Test stopping program execution with SIGKILL."""
        self.http_client.patch.return_value = None

        self.programs.stop(sigkill=True)

        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.STOPPED)
        self.assertTrue(request.sigkill)

    def test_resume(self):
        """Test resuming program execution."""
        self.http_client.patch.return_value = None

        self.programs.resume()

        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.RUNNING)

    def test_on_execution_output(self):
        """Test subscribing to execution output."""
        callback = MagicMock()

        self.programs.on_execution_output(base_program.id, base_execution.id, callback)

        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        expected_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_OUTPUT,
                                                 program_id=base_program.id,
                                                 execution_id=base_execution.id)
        self.assertEqual(topic, expected_topic)
        self.assertEqual(model, transport_models.a.ProgramExecutionOutput)

        # Simulate receiving output
        internal_callback(topic, transport_models.a.ProgramExecutionOutput(output='test output', eof=False))
        callback.assert_called_once_with('test output', False)

    def test_on_execution_status(self):
        """Test subscribing to execution status changes."""
        callback = MagicMock()

        self.programs.on_execution_status(base_program.id, base_execution.id, callback)

        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, _ = self.mqtt_client.subscribe.call_args[0]
        expected_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_STATUS,
                                                 program_id=base_program.id,
                                                 execution_id=base_execution.id)
        self.assertEqual(topic, expected_topic)


class TestExecutionCommand(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.programs = Programs(http_client=self.http_client,
                                 mqtt_client=self.mqtt_client,
                                 correlation_id=self.correlation_id)

    def tearDown(self):
        del self.programs

    def _make_cmd(self, on_output=None, on_status=None, timeout=-1):
        execution_id = str(uuid4())
        cmd = ExecutionBaseCommand(self.programs, base_program.id, execution_id, timeout, on_output, on_status)
        return cmd, execution_id

    def test_status_property_initial(self):
        """An execution starts pending: the robot has taken the request and the program has not run a line yet."""
        cmd, _ = self._make_cmd()
        self.assertEqual(cmd.status, ExecutorStatus.PENDING)

    def test_execution_id_property(self):
        cmd, execution_id = self._make_cmd()
        self.assertEqual(cmd.execution_id, execution_id)

    def test_output_eof_unsubscribes_the_output_not_the_status(self):
        unsubscribe_output, unsubscribe_status = MagicMock(), MagicMock()
        self.mqtt_client.subscribe.side_effect = [unsubscribe_output, unsubscribe_status]
        cmd, execution_id = self._make_cmd(on_output=MagicMock())
        output_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_OUTPUT,
                                               program_id=base_program.id,
                                               execution_id=execution_id)
        internal_callback = self.mqtt_client.subscribe.call_args_list[0][0][1]

        internal_callback(output_topic, transport_models.a.ProgramExecutionOutput(output='bye', eof=True))

        unsubscribe_output.assert_called_once()
        unsubscribe_status.assert_not_called()

    def _feed_status(self, cmd, execution_id, status: ExecutorStatus):
        internal_callback = self.mqtt_client.subscribe.call_args_list[-1][0][1]
        execution = programs.ProgramExecution(
            id=execution_id,
            program_id=base_program.id,
            context=programs.ProgramExecutionContext(environment={}, arguments=[]),
            started_at=datetime.now(),
            finished_at=None,
            exit_code=None,
            status=status,
        )
        internal_callback(
            'topic',
            transport_models.a.ProgramsExecutorStatus(
                status=transport_models.a.ProgramsExecutorStatusStatus(status.value),
                execution=execution.to_transport_model().model_dump(),
            ),
        )

    def test_wait_raises_when_the_execution_is_stopped(self):
        from pyniryo.nate.exceptions import ExecutionStoppedError
        for status in (ExecutorStatus.STOPPED, ExecutorStatus.SAFETY_STOPPED):
            with self.subTest(status=status):
                cmd, execution_id = self._make_cmd()
                self._feed_status(cmd, execution_id, status)
                with self.assertRaises(ExecutionStoppedError):
                    cmd.wait()

    def test_status_callback_does_not_fetch_the_execution(self):
        cmd, execution_id = self._make_cmd()

        self._feed_status(cmd, execution_id, ExecutorStatus.RUNNING)

        self.http_client.get.assert_not_called()

    def test_execution_is_fetched_on_demand(self):
        cmd, execution_id = self._make_cmd()
        self.http_client.get.return_value = base_execution.to_transport_model()

        execution = cmd.execution

        self.http_client.get.assert_called_once_with(
            paths_gen.Programs.GET_PROGRAM_EXECUTION.format(program_id=base_program.id, execution_id=execution_id),
            transport_models.s.ProgramExecution)
        self.assertEqual(execution.id, base_execution.id)

    def test_program_id_property(self):
        """Test that program_id property returns the correct value."""
        cmd, _ = self._make_cmd()
        self.assertEqual(cmd.program_id, base_program.id)

    def test_output_callback_invocation(self):
        """Test that output callback is invoked when output is received."""
        output_callback = MagicMock()
        cmd, execution_id = self._make_cmd(on_output=output_callback)

        # Get the internal callback registered for the output topic
        output_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_OUTPUT,
                                               program_id=base_program.id,
                                               execution_id=execution_id)

        internal_callback = None
        for call_args in self.mqtt_client.subscribe.call_args_list:
            if call_args[0][0] == output_topic:
                internal_callback = call_args[0][1]
                break

        self.assertIsNotNone(internal_callback)

        # Simulate receiving output
        internal_callback(output_topic, transport_models.a.ProgramExecutionOutput(output='test output', eof=False))
        output_callback.assert_called_once_with('test output', False)

    def test_status_callback_invocation(self):
        """Test that status callback is invoked when status changes."""
        status_callback = MagicMock()
        cmd, execution_id = self._make_cmd(on_status=status_callback)

        # Get the internal status callback
        status_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_STATUS,
                                               program_id=base_program.id,
                                               execution_id=execution_id)

        internal_callback = None
        for call_args in self.mqtt_client.subscribe.call_args_list:
            if call_args[0][0] == status_topic:
                internal_callback = call_args[0][1]
                break

        self.assertIsNotNone(internal_callback)

        # Build an execution whose id matches the cmd's execution_id so the filter passes
        matching_execution = programs.ProgramExecution(
            id=execution_id,
            program_id=base_program.id,
            context=programs.ProgramExecutionContext(environment={}, arguments=[]),
            started_at=datetime.now(),
            finished_at=None,
            exit_code=None,
            status=ExecutorStatus.COMPLETED,
        )
        self.http_client.get.return_value = matching_execution.to_transport_model()
        internal_callback(
            status_topic,
            transport_models.a.ProgramsExecutorStatus(
                status=transport_models.a.ProgramsExecutorStatusStatus.COMPLETED,
                execution=matching_execution.to_transport_model().model_dump(),
            ),
        )

        status_callback.assert_called_once_with(ExecutorStatus.COMPLETED)

    def _get_status_internal_callback(self, cmd, execution_id):
        """Helper to retrieve the internal status callback registered via MQTT subscribe."""
        status_topic = self.mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_STATUS,
                                               program_id=base_program.id,
                                               execution_id=execution_id)
        for call_args in self.mqtt_client.subscribe.call_args_list:
            if call_args[0][0] == status_topic:
                return call_args[0][1], status_topic
        return None, None

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_success(self, mock_monotonic, mock_sleep):
        """Test waiting for execution to complete successfully."""
        mock_monotonic.return_value = 0.0

        cmd, execution_id = self._make_cmd()

        # Build an execution whose id matches execution_id so the callback filter passes
        matching_execution = programs.ProgramExecution(
            id=execution_id,
            program_id=base_program.id,
            context=programs.ProgramExecutionContext(environment={}, arguments=[]),
            started_at=datetime.now(),
            finished_at=datetime.now(),
            exit_code=0,
            status=ExecutorStatus.COMPLETED,
        )
        self.http_client.get.return_value = matching_execution.to_transport_model()

        # Simulate status change to COMPLETED
        internal_callback, status_topic = self._get_status_internal_callback(cmd, execution_id)
        self.assertIsNotNone(internal_callback)

        internal_callback(
            status_topic,
            transport_models.a.ProgramsExecutorStatus(
                status=transport_models.a.ProgramsExecutorStatusStatus.COMPLETED,
                execution=matching_execution.to_transport_model().model_dump(),
            ),
        )

        # Wait should return without error
        cmd.wait()

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_timeout(self, mock_monotonic, mock_sleep):
        """Test waiting for execution with timeout."""
        mock_monotonic.side_effect = [0.0, 1.0, 2.0, 3.0]

        cmd, _ = self._make_cmd(timeout=1.0)

        # Status stays RUNNING → should time out
        with self.assertRaises(TimeoutError):
            cmd.wait()

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_failure(self, mock_monotonic, mock_sleep):
        """Test waiting for execution that fails."""
        mock_monotonic.return_value = 0.0

        cmd, execution_id = self._make_cmd()

        # Build an execution whose id matches execution_id so the callback filter passes
        matching_execution = programs.ProgramExecution(
            id=execution_id,
            program_id=base_program.id,
            context=programs.ProgramExecutionContext(environment={}, arguments=[]),
            started_at=datetime.now(),
            finished_at=datetime.now(),
            exit_code=1,
            status=ExecutorStatus.FAILED,
        )
        self.http_client.get.return_value = matching_execution.to_transport_model()

        # Simulate status change to FAILED
        internal_callback, status_topic = self._get_status_internal_callback(cmd, execution_id)
        self.assertIsNotNone(internal_callback)

        internal_callback(
            status_topic,
            transport_models.a.ProgramsExecutorStatus(
                status=transport_models.a.ProgramsExecutorStatusStatus.FAILED,
                execution=matching_execution.to_transport_model().model_dump(),
            ),
        )

        # Wait should raise PyNiryoError
        with self.assertRaises(PyNiryoError):
            cmd.wait()


if __name__ == "__main__":
    unittest.main()


class TestProgramModels(unittest.TestCase):

    def test_golang_program_type(self):
        program = programs.Program.from_transport_model(
            transport_models.s.Program(id=uuid4(), name='svc', type=transport_models.s.ProgramType.GOLANG))
        self.assertEqual(program.type, programs.ProgramType.GOLANG)

    def test_python_picks_the_latest_python_version_only(self):
        self.assertEqual(programs.ProgramType.python(), programs.ProgramType.PYTHON312)

    def test_execution_carries_the_program_name(self):
        model = base_execution.to_transport_model()
        model.programName = 'pick-and-place'
        self.assertEqual(programs.ProgramExecution.from_transport_model(model).program_name, 'pick-and-place')

    def test_executor_status_final_states(self):
        for status in (ExecutorStatus.COMPLETED, ExecutorStatus.FAILED, ExecutorStatus.STOPPED,
                       ExecutorStatus.SAFETY_STOPPED):
            self.assertTrue(status.is_final(), status)
        for status in (ExecutorStatus.IDLE, ExecutorStatus.PENDING, ExecutorStatus.RUNNING, ExecutorStatus.PAUSED,
                       ExecutorStatus.SAFETY_PAUSED):
            self.assertFalse(status.is_final(), status)

    def test_executor_status_is_stopped(self):
        self.assertTrue(ExecutorStatus.STOPPED.is_stopped())
        self.assertTrue(ExecutorStatus.SAFETY_STOPPED.is_stopped())
        self.assertFalse(ExecutorStatus.FAILED.is_stopped())
