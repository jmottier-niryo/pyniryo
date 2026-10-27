from dataclasses import dataclass
from datetime import datetime

from .._internal import transport_models
from .programs import Program


@dataclass
class Project:
    """
    A named group of programs. A program belongs to at most one project, and a program outside any project stays
    valid. Deleting a project also deletes the programs it contains.

    :param id: The unique identifier of the project.
    :param name: Unique on the robot. At most 255 bytes, without ``/``, and not ``.`` or ``..``.
    :param description: Free text shown with the project, empty when none was given.
    :param active: Whether this is the robot's active project. At most one project is active.
    :param updated_at: Last change to the project or to a program it contains.
    :param programs: The programs the project contains.
    """
    id: str
    name: str
    description: str
    active: bool
    updated_at: datetime
    programs: list[Program]

    @classmethod
    def from_transport_model(cls, model: transport_models.s.Project) -> 'Project':
        return cls(id=str(model.id),
                   name=model.name,
                   description=model.description,
                   active=model.active,
                   updated_at=model.updated_at,
                   programs=[Program.from_transport_model(p) for p in model.programs])
