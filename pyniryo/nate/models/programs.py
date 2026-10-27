import re
from dataclasses import dataclass
from datetime import datetime
from typing import Optional
from uuid import UUID

from strenum import StrEnum

from .._internal import transport_models
from . import ExecutorStatus


class ProgramType(StrEnum):
    """
    Enumeration of supported program types and Python versions.
    """
    PYTHON39 = 'python3.9'
    PYTHON310 = 'python3.10'
    PYTHON311 = 'python3.11'
    PYTHON312 = 'python3.12'
    GOLANG = 'golang'

    def _key(self) -> tuple[str, tuple[int, ...]]:
        pattern = r'^([a-z]+)([\d\.?]+)$'
        match = re.match(pattern, self)
        if not match:
            raise ValueError(f"Invalid ProgramType format: {self}")
        return match.group(1), tuple(int(v) for v in match.group(2).split('.'))

    @classmethod
    def python(cls) -> 'ProgramType':
        """
        Get the latest supported Python version.
        :return: The latest supported Python version.
        """
        return max((e for e in cls if e.value.startswith('python')), key=lambda e: e._key())

    @classmethod
    def from_transport_model(cls, model: transport_models.s.ProgramType) -> 'ProgramType':
        return cls(model.value)

    def to_transport_model(self) -> transport_models.s.ProgramType:
        return transport_models.s.ProgramType(str(self))


@dataclass
class Program:
    """
    Represents a program stored on the robot.
    
    :param id: The unique identifier of the program.
    :param name: The name of the program.
    :param type: The type/version of the program.
    """
    id: str
    name: str
    type: ProgramType

    @classmethod
    def from_transport_model(cls, model: transport_models.s.Program) -> 'Program':
        return cls(
            id=str(model.id),
            name=model.name,
            type=ProgramType.from_transport_model(model.type),
        )

    def to_transport_model(self) -> transport_models.s.Program:
        return transport_models.s.Program(
            id=UUID(self.id),
            name=self.name,
            type=transport_models.s.ProgramType(self.type.value),
        )


@dataclass
class ProgramExecutionContext:
    """
    Represents the execution context for a program.
    
    :param environment: Environment variables to set for the program execution.
    :param arguments: Command-line arguments to pass to the program.
    """
    environment: dict[str, str]
    arguments: list[str]

    @classmethod
    def from_transport_model(cls, model: transport_models.s.ProgramExecutionContext) -> 'ProgramExecutionContext':
        return cls(
            environment=model.environment or {},
            arguments=model.arguments or [],
        )

    def to_transport_model(self) -> transport_models.s.ProgramExecutionContext:
        return transport_models.s.ProgramExecutionContext(environment=self.environment, arguments=self.arguments)


@dataclass
class ExecutionOutput:
    """
    Represents output from a program execution.
    
    :param output: The output text from the program.
    :param eof: Whether this is the end of the output stream.
    """
    output: str
    eof: bool

    @classmethod
    def from_transport_model(cls, model: transport_models.a.ProgramExecutionOutput) -> 'ExecutionOutput':
        return cls(output=model.output, eof=model.eof)

    def to_transport_model(self) -> transport_models.a.ProgramExecutionOutput:
        return transport_models.a.ProgramExecutionOutput(output=self.output, eof=self.eof)


ExecutionStatusStatus = ExecutorStatus


@dataclass
class ProgramExecution:
    """
    Represents a program execution instance.

    :param id: The unique identifier of the execution.
    :param program_id: The ID of the program being executed.
    :param program_name: The name of the program being executed, when the robot reports it.
    :param context: The execution context (environment variables and arguments).
    :param started_at: When the execution started.
    :param finished_at: When the execution finished.
    :param exit_code: The exit code of the program execution.
    """
    id: str
    program_id: str
    context: ProgramExecutionContext
    started_at: datetime | None
    finished_at: datetime | None
    exit_code: int | None
    status: ExecutionStatusStatus
    program_name: str | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.ProgramExecution) -> 'ProgramExecution':
        return cls(id=str(model.id),
                   program_id=str(model.programId),
                   program_name=model.programName,
                   context=ProgramExecutionContext.from_transport_model(model.context),
                   started_at=model.startedAt,
                   finished_at=model.finishedAt,
                   exit_code=model.exitCode,
                   status=ExecutionStatusStatus(model.status.value))

    def to_transport_model(self) -> transport_models.s.ProgramExecution:
        return transport_models.s.ProgramExecution(id=UUID(self.id),
                                                   programId=UUID(self.program_id),
                                                   programName=self.program_name,
                                                   context=self.context.to_transport_model(),
                                                   startedAt=self.started_at,
                                                   finishedAt=self.finished_at,
                                                   exitCode=self.exit_code,
                                                   status=transport_models.s.ExecutorStatus(self.status))


@dataclass
class ProgramExecutorStatus:
    """
    Represents the current status of the program executor.

    :param execution: The current execution (None if no execution).
    :param status: The current execution status.
    """
    execution: Optional[ProgramExecution]
    status: ExecutorStatus

    @classmethod
    def from_transport_model(cls, model: transport_models.s.ProgramsExecutorStatus) -> 'ProgramExecutorStatus':
        return cls(status=ExecutionStatusStatus.from_transport_model(model.status),
                   execution=ProgramExecution.from_transport_model(model.execution) if model.execution else None)

    @classmethod
    def from_a_transport_model(cls, model: transport_models.a.ProgramsExecutorStatus) -> 'ProgramExecutorStatus':
        return cls(
            status=ExecutionStatusStatus.from_transport_model(transport_models.s.ExecutorStatus(model.status.value)),
            execution=ProgramExecution.from_transport_model(
                transport_models.s.ProgramExecution.model_validate(model.execution)) if model.execution else None,
        )

    def to_transport_model(self) -> transport_models.s.ProgramsExecutorStatus:
        return transport_models.s.ProgramsExecutorStatus(
            status=self.status.to_transport_model(),
            execution=self.execution.to_transport_model() if self.execution else None)
