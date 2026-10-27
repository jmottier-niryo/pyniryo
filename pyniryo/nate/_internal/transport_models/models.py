from typing import Any

from pydantic import RootModel, BaseModel, Field

from .models_gen import User, Token, TokenInfo, Program, ProgramExecution, InterfaceState, Project

UserList = RootModel[list[User]]
TokenList = RootModel[list[Token]]
TokenInfoList = RootModel[list[TokenInfo]]

ProjectList = RootModel[list[Project]]

ProgramList = RootModel[list[Program]]
ProgramExecutionList = RootModel[list[ProgramExecution]]

FrameIdList = RootModel[list[str]]

InterfaceStateList = RootModel[list[InterfaceState]]


class HealthCheckResponse(BaseModel):
    status: str | None = Field(None, description="The health status of the robot.", examples=['ok'])


class ReadinessCheckResponse(BaseModel):
    ready: bool | None = Field(None, description="Whether the robot is ready or not.")


class NTPConfigUpdate(BaseModel):
    """
    The writable part of the NTP configuration: synchronized and last_sync are read only.
    """
    enabled: bool
    servers: list[str]
    pools: list[str]


class JsonObject(RootModel[dict[str, Any]]):
    """
    A payload the specification leaves as a bare object.
    """
    root: dict[str, Any]


class EmptyPayload(BaseModel):
    """
    An empty response model for endpoints that return no data.
    """
    pass
