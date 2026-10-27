from typing import IO

from .base_api_component import BaseAPIComponent
from ..models import Program, ProgramType, Project
from .._internal import paths_gen, transport_models


class Projects(BaseAPIComponent):
    """
    Projects component for grouping programs. A program belongs to at most one project, and at most one project is
    active on the robot.
    """

    def get_all(self) -> list[Project]:
        """
        Get all the projects, each carrying the programs it contains.

        :return: The list of all the projects.
        """
        projects = self._http_client.get(paths_gen.Projects.GET_ALL_PROJECTS, transport_models.ProjectList)
        return [Project.from_transport_model(project) for project in projects.root]

    def get(self, project_id: str) -> Project:
        """
        Get a project by its ID.

        :param project_id: The ID of the project.
        :return: The corresponding project, with the programs it contains.
        :raises ClientError: If no project has this ID.
        """
        project = self._http_client.get(paths_gen.Projects.GET_PROJECT.format(project_id=project_id),
                                        transport_models.s.Project)
        return Project.from_transport_model(project)

    def create(self, name: str, description: str | None = None) -> Project:
        """
        Create a project, with no program in it yet.

        :param name: The name of the project, unique on the robot. At most 255 bytes, without ``/``, and not ``.``
            or ``..``.
        :param description: Free text shown with the project. If None, the project starts with an empty one.
        :return: The created project.
        :raises ClientError: If the name is invalid, or a project with this name already exists.
        """
        project = self._http_client.post(paths_gen.Projects.CREATE_PROJECT,
                                         transport_models.s.Project,
                                         transport_models.s.NewProject(name=name, description=description))
        return Project.from_transport_model(project)

    def update(self, project_id: str, name: str, description: str | None = None) -> Project:
        """
        Update the name or the description of a project.

        :param project_id: The ID of the project.
        :param name: The new name of the project, same rules as :meth:`create`.
        :param description: The new description. If None, the stored one stays.
        :return: The updated project.
        :raises ClientError: If no project has this ID, the name is invalid, or another project has this name.
        """
        project = self._http_client.patch(paths_gen.Projects.UPDATE_PROJECT.format(project_id=project_id),
                                          transport_models.s.Project,
                                          transport_models.s.NewProject(name=name, description=description))
        return Project.from_transport_model(project)

    def delete(self, project_id: str) -> None:
        """
        Delete a project. This also deletes the programs the project contains.

        :param project_id: The ID of the project.
        :raises ClientError: If no project has this ID.
        """
        self._http_client.delete(paths_gen.Projects.DELETE_PROJECT.format(project_id=project_id),
                                 transport_models.EmptyPayload)

    def activate(self, project_id: str) -> None:
        """
        Make a project the robot's active project, deactivating the previous one. Activating the project that is
        already active changes nothing.

        :param project_id: The ID of the project.
        :raises ClientError: If no project has this ID.
        """
        self._http_client.put(paths_gen.Projects.ACTIVATE_PROJECT.format(project_id=project_id),
                              transport_models.EmptyPayload,
                              transport_models.EmptyPayload())

    def create_program(self,
                       project_id: str,
                       program: IO[bytes],
                       program_type: ProgramType,
                       name: str | None = None) -> Program:
        """
        Upload a new program inside a project.

        :param project_id: The ID of the project.
        :param program: A file-like object containing the program content.
        :param program_type: The type of the program.
        :param name: The name of the program. If None, the robot uses the file name.
        :return: The created program.
        :raises ClientError: If no project has this ID, or the project already contains a program with this name.
        """
        response = self._http_client.post(
            paths_gen.Projects.CREATE_PROGRAM_IN_PROJECT.format(project_id=project_id),
            transport_models.s.Program,
            transport_models.s.UploadProgram(name=name, type=program_type.to_transport_model(), file=bytes()),
            files={'file': program},
            exclude={'file'},
        )
        return Program.from_transport_model(response)

    def add_program(self, project_id: str, program_id: str) -> None:
        """
        Move a program into a project. The program leaves the project it was in, if any. Moving a program into the
        project it is already in changes nothing.

        :param project_id: The ID of the project.
        :param program_id: The ID of the program.
        :raises ClientError: If no project or no program has this ID, or the project already contains a program
            with this name.
        """
        self._http_client.put(
            paths_gen.Projects.ADD_PROGRAM_TO_PROJECT.format(project_id=project_id, program_id=program_id),
            transport_models.EmptyPayload,
            transport_models.EmptyPayload())

    def remove_program(self, project_id: str, program_id: str) -> None:
        """
        Take a program out of its project. The program is kept and stays usable outside any project.

        :param project_id: The ID of the project.
        :param program_id: The ID of the program.
        :raises ClientError: If no project or no program has this ID, or the program is not in this project.
        """
        self._http_client.delete(
            paths_gen.Projects.REMOVE_PROGRAM_FROM_PROJECT.format(project_id=project_id, program_id=program_id),
            transport_models.EmptyPayload)
