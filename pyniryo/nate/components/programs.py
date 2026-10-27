import logging
import time
from typing import IO, Callable
from uuid import uuid4, UUID

from .base_api_component import BaseAPIComponent, BaseCommand
from ..exceptions import PyNiryoError, ExecutionStoppedError
from ..models import Program, ProgramType, ProgramExecution, ExecutionStatusStatus, ProgramExecutorStatus, Unsubscribe, ExecutionOutput
from .._internal import paths_gen, transport_models, topics_gen

logger = logging.getLogger(__name__)

ExecutionOutputCallback = Callable[[str, bool], None]
ExecutionStatusCallback = Callable[[ExecutionStatusStatus], None]


class Programs(BaseAPIComponent):
    """
    Programs component for managing and executing robot programs.
    
    This component provides methods for:
    - Creating, uploading, and managing programs
    - Executing programs with custom environment variables and arguments
    - Monitoring program execution status and output
    - Controlling program execution (pause, resume, stop)
    """

    def get_all(self) -> list[Program]:
        """
        Get all the registered programs.

        :return: The list of all the programs.
        """
        programs = self._http_client.get(
            paths_gen.Programs.GET_ALL_PROGRAMS,
            transport_models.ProgramList,
        )
        return [Program.from_transport_model(program) for program in programs.root]

    def create(self, name: str | None, program: IO[bytes], program_type: ProgramType) -> Program:
        """
        Upload a new program.

        :param name: The name of the program. If None, the robot uses the file name.
        :param program: A file-like object containing the program content.
        :param program_type: The type of the program.
        :return: The created program.
        """
        response = self._http_client.post(
            paths_gen.Programs.CREATE_PROGRAM,
            transport_models.s.Program,
            transport_models.s.UploadProgram(name=name, type=program_type.to_transport_model(), file=bytes()),
            files={'file': program},
            exclude={'file'},
        )
        return Program.from_transport_model(response)

    def get(self, program_id: str, dst: IO[bytes] | None = None) -> Program:
        """
        Get a program by its ID.

        :param program_id: The ID of the program.
        :param dst: Optional file-like object to download the program content into. If None, the content is not downloaded.
        :return: The corresponding program.
        """
        tr_program = self._http_client.get(
            paths_gen.Programs.GET_PROGRAM.format(program_id=program_id),
            transport_models.s.Program,
        )
        program = Program.from_transport_model(tr_program)
        if dst is not None:
            self._http_client.download(paths_gen.Programs.GET_PROGRAM_FILE.format(program_id=program_id), dst)
        return program

    def delete(self, program_id: str) -> None:
        """
        Delete a program by its ID.

        :param program_id: The ID of the program.
        :return: None
        """
        self._http_client.delete(
            paths_gen.Programs.DELETE_PROGRAM.format(program_id=program_id),
            transport_models.EmptyPayload,
        )

    def update(self, program: Program, src: IO[bytes] | None = None) -> Program:
        """
        Update a program. The metadata and the content are two requests: a content upload that fails leaves the
        metadata already changed.

        :param program: The program to update.
        :param src: Optional file-like object containing the new program content. If None, the content is not updated.
        :return: The updated program.
        """
        response = self._http_client.patch(
            paths_gen.Programs.UPDATE_PROGRAM.format(program_id=program.id),
            transport_models.s.Program,
            program.to_transport_model(),
        )
        if src is not None:
            self._http_client.patch(paths_gen.Programs.UPLOAD_PROGRAM_FILE.format(program_id=program.id),
                                    transport_models.EmptyPayload,
                                    transport_models.EmptyPayload(),
                                    files={'file': src})
        return Program.from_transport_model(response)

    def get_executions(self, program_id: str) -> list[ProgramExecution]:
        """
        Get all the executions of a program.

        :param program_id: The ID of the program.
        :return: The list of all the executions of the program.
        """
        executions = self._http_client.get(
            paths_gen.Programs.GET_PROGRAM_EXECUTIONS.format(program_id=program_id),
            transport_models.ProgramExecutionList,
        )
        return [ProgramExecution.from_transport_model(execution) for execution in executions.root]

    def get_execution(self, program_id: str, execution_id: str) -> ProgramExecution:
        """
        Get a program execution by its ID.

        :param program_id: The ID of the program.
        :param execution_id: The ID of the program execution.
        :return: The corresponding program execution.
        """
        execution = self._http_client.get(
            paths_gen.Programs.GET_PROGRAM_EXECUTION.format(program_id=program_id, execution_id=execution_id),
            transport_models.s.ProgramExecution,
        )
        return ProgramExecution.from_transport_model(execution)

    def get_execution_output(self, program_id: str, execution_id: str, dst: IO[bytes]) -> None:
        """
        Download the full output of a program execution.

        :param program_id: The ID of the program.
        :param execution_id: The ID of the execution.
        :param dst: A file-like object to write the output content into.
        """
        self._http_client.download(
            paths_gen.Programs.GET_PROGRAM_EXECUTION_OUTPUT.format(program_id=program_id, execution_id=execution_id),
            dst)

    def execute(self,
                program_id: str,
                environment: dict[str, str] | None = None,
                arguments: list[str] | None = None,
                timeout: float = -1,
                on_output: ExecutionOutputCallback | None = None,
                on_status: ExecutionStatusCallback | None = None) -> 'ExecutionBaseCommand':
        """
        Execute a program.

        :param program_id: The ID of the program.
        :param environment: Optional dictionary of environment variables to pass to the program.
        :param arguments: Optional list of command line arguments to pass to the program.
        :param timeout: Maximum time in seconds to wait before considering the program failed.
            A negative value means no timeout.
        :param on_output: Optional callback to listen to the program outputs. The callback must take the output string
            and a boolean indicating if it's the end of the output as parameters.
        :param on_status: Optional callback to listen the program execution status changes.
            The callback must take the new status as a parameter.
        :return: The created program execution command.
        :raises ClientError: If another workstation holds the robot (``code == 'robot_held'``) or an execution is
            already running.
        """
        execution_id = str(uuid4())
        execution_command = ExecutionBaseCommand(self, program_id, execution_id, timeout, on_output, on_status)

        try:
            self._http_client.post(
                paths_gen.Programs.EXECUTE_PROGRAM.format(program_id=program_id),
                transport_models.s.FeedbackResponse,
                transport_models.s.ExecuteProgram(execution_id=UUID(execution_id),
                                                  context=transport_models.s.ProgramExecutionContext(
                                                      environment=environment, arguments=arguments)),
            )
        except Exception:
            execution_command._stop_tracking()
            raise
        return execution_command

    def executor_status(self) -> ProgramExecutorStatus:
        """
        Get the current status of the program executor.
        
        :return: The current status of the program executor.
        """
        status = self._http_client.get(paths_gen.Programs.GET_PROGRAMS_EXECUTOR_STATUS,
                                       transport_models.s.ProgramsExecutorStatus)
        return ProgramExecutorStatus.from_transport_model(status)

    def _update_executor_status(self, status: transport_models.s.ExecutorStatus, sigkill: bool = False) -> None:
        self._http_client.patch(paths_gen.Programs.UPDATE_PROGRAMS_EXECUTOR_STATUS,
                                transport_models.EmptyPayload,
                                transport_models.s.ProgramsExecutorStatus(status=status, sigkill=sigkill))

    def pause(self) -> None:
        """
        Pause the current program execution. A pause is taken from the moment the execution is requested, the
        program's own startup included: a program paused before it runs a line carries on its startup once resumed.
        A pause asked by a client holds until :meth:`resume` is called, a protective stop pauses and resumes on
        its own.

        :raises ClientError: If nothing is running (``code == 'no_execution'``) or the execution is in a state a
            pause does not follow from.
        """
        self._update_executor_status(transport_models.s.ExecutorStatus.PAUSED)

    def stop(self, sigkill: bool = False) -> None:
        """
        Stop the current program execution. A stop is taken from the moment the execution is requested, the
        program's own startup included: a program stopped before it runs a line never runs one.

        :param sigkill: If True, force kill the program. If False, send a graceful stop signal.
        :raises ClientError: If nothing is running (``code == 'no_execution'``).
        """
        self._update_executor_status(transport_models.s.ExecutorStatus.STOPPED, sigkill=sigkill)

    def resume(self) -> None:
        """
        Resume a program execution paused by :meth:`pause`.

        :raises ClientError: If nothing is running (``code == 'no_execution'``), the execution is not paused, or
            the safety chain closes the resume.
        """
        self._update_executor_status(transport_models.s.ExecutorStatus.RUNNING)

    def on_execution_output(self, program_id: str, execution_id: str, callback: ExecutionOutputCallback) -> Unsubscribe:

        def internal_callback(_: str, payload: transport_models.a.ProgramExecutionOutput) -> None:
            exec_out = ExecutionOutput.from_transport_model(payload)
            callback(exec_out.output, exec_out.eof)

        topic = self._mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_OUTPUT,
                                         program_id=program_id,
                                         execution_id=execution_id)
        return self._mqtt_client.subscribe(topic, internal_callback, transport_models.a.ProgramExecutionOutput)

    def on_execution_status(self, program_id: str, execution_id: str, callback: ExecutionStatusCallback) -> Unsubscribe:

        def internal_callback(_: str, payload: transport_models.a.ProgramsExecutorStatus) -> None:
            exec_status = ProgramExecutorStatus.from_a_transport_model(payload)
            if exec_status.execution is None or exec_status.execution.id != execution_id:
                return
            callback(exec_status.execution.status)

        topic = self._mqtt_client.format(topics_gen.Programs.PROGRAM_EXECUTION_STATUS,
                                         program_id=program_id,
                                         execution_id=execution_id)
        return self._mqtt_client.subscribe(topic, internal_callback, transport_models.a.ProgramsExecutorStatus)


class ExecutionBaseCommand(BaseCommand):
    """
    Represents a program execution command. Monitors the output and status during the execution.
    """

    def __init__(self,
                 programs: Programs,
                 program_id: str,
                 execution_id: str,
                 timeout: float,
                 on_output: ExecutionOutputCallback | None,
                 on_status: ExecutionStatusCallback | None) -> None:
        super().__init__(execution_id, timeout)
        self._programs: Programs = programs
        self._program_id: str = program_id
        self._user_on_output = on_output
        self._user_on_status = on_status

        self._unsubscribe_output: Unsubscribe = lambda: None
        if self._user_on_output is not None:
            self._unsubscribe_output = self._programs.on_execution_output(self.program_id,
                                                                          execution_id,
                                                                          self._output_callback)

        self._status: list[ExecutionStatusStatus] = [ExecutionStatusStatus.PENDING]
        self._unsubscribe_status = self._programs.on_execution_status(program_id, execution_id, self._status_callback)

    def _stop_tracking(self) -> None:
        self._unsubscribe_output()
        self._unsubscribe_status()

    def _output_callback(self, output: str, eof: bool) -> None:
        """
        Internal callback to handle execution output messages.
        """
        if self._user_on_output is not None:
            try:
                self._user_on_output(output, eof)
            except Exception as e:
                logger.error(f'Error in on_output callback: {e}')
        if eof:
            self._unsubscribe_output()

    def _status_callback(self, status: ExecutionStatusStatus) -> None:
        """
        Internal callback to handle execution state messages.
        """
        if self._user_on_status is not None:
            try:
                self._user_on_status(status)
            except Exception as e:
                logger.error(f'Error in on_status callback: {e}')

        self._status.append(status)
        if self.status.is_final():
            self._unsubscribe_status()

    @property
    def status(self) -> ExecutionStatusStatus:
        """
        Get the current status of the execution.

        :return: The current status of the execution.
        """
        return self._status[-1]

    @property
    def execution(self) -> ProgramExecution:
        """
        Fetch the execution from the robot, with its context, timestamps and exit code.

        :return: The execution as the robot reports it now.
        """
        return self._programs.get_execution(self._program_id, self._command_id)

    @property
    def program_id(self) -> str:
        return self._program_id

    @property
    def execution_id(self) -> str:
        return self._command_id

    def wait(self) -> None:
        """
        Wait for the program execution to complete. A paused execution keeps waiting until it is resumed, and the
        time spent paused counts against the timeout.

        :raises TimeoutError: If the timeout is exceeded.
        :raises ExecutionStoppedError: If the execution was stopped by a client or by the safety chain.
        :raises PyNiryoError: If the execution fails.
        """
        while not self.status.is_final():
            if self._timeout > 0 and self._started_at + self._timeout < time.monotonic():
                raise TimeoutError(f'Execution {self._command_id} timed out after {self._timeout} seconds.')
            time.sleep(0.1)
        if self.status.is_stopped():
            raise ExecutionStoppedError(f'Execution {self._command_id} was stopped ({self.status})')
        if self.status.is_error():
            raise PyNiryoError(f'Execution {self._command_id} failed')
