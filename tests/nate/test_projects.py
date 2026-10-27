import unittest
from datetime import datetime, timezone
from io import BytesIO
from uuid import uuid4

from pyniryo.nate._internal import transport_models, paths_gen
from pyniryo.nate.components.projects import Projects
from pyniryo.nate.models import programs
from pyniryo.nate.models.projects import Project

from .base import BaseTestComponent

base_program = programs.Program(id=str(uuid4()), name='pick', type=programs.ProgramType.PYTHON312)

t_project = transport_models.s.Project(id=uuid4(),
                                       name='Palletizing',
                                       description='Boxes',
                                       active=True,
                                       updated_at=datetime(2026, 9, 1, tzinfo=timezone.utc),
                                       programs=[base_program.to_transport_model()])

expected_project = Project(id=str(t_project.id),
                           name='Palletizing',
                           description='Boxes',
                           active=True,
                           updated_at=datetime(2026, 9, 1, tzinfo=timezone.utc),
                           programs=[base_program])


class TestProjects(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.projects = Projects(http_client=self.http_client,
                                 mqtt_client=self.mqtt_client,
                                 correlation_id=self.correlation_id)

    def test_get_all(self):
        self.http_client.get.return_value = transport_models.ProjectList([t_project])

        all_projects = self.projects.get_all()

        self.http_client.get.assert_called_once_with(paths_gen.Projects.GET_ALL_PROJECTS, transport_models.ProjectList)
        self.assertEqual(all_projects, [expected_project])

    def test_get(self):
        self.http_client.get.return_value = t_project

        project = self.projects.get(str(t_project.id))

        self.http_client.get.assert_called_once_with(paths_gen.Projects.GET_PROJECT.format(project_id=t_project.id),
                                                     transport_models.s.Project)
        self.assertEqual(project, expected_project)

    def test_create(self):
        self.http_client.post.return_value = t_project

        project = self.projects.create('Palletizing', description='Boxes')

        self.http_client.post.assert_called_once_with(paths_gen.Projects.CREATE_PROJECT,
                                                      transport_models.s.Project,
                                                      transport_models.s.NewProject(name='Palletizing',
                                                                                    description='Boxes'))
        self.assertEqual(project, expected_project)

    def test_create_without_a_description_leaves_it_out(self):
        self.http_client.post.return_value = t_project

        self.projects.create('Palletizing')

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(request.model_dump(exclude_none=True), {'name': 'Palletizing'})

    def test_update(self):
        self.http_client.patch.return_value = t_project

        project = self.projects.update(str(t_project.id), name='Palletizing')

        self.http_client.patch.assert_called_once_with(
            paths_gen.Projects.UPDATE_PROJECT.format(project_id=t_project.id), transport_models.s.Project,
            transport_models.s.NewProject(name='Palletizing'))
        self.assertEqual(project, expected_project)

    def test_delete(self):
        self.http_client.delete.return_value = transport_models.EmptyPayload()

        self.projects.delete(str(t_project.id))

        self.http_client.delete.assert_called_once_with(
            paths_gen.Projects.DELETE_PROJECT.format(project_id=t_project.id), transport_models.EmptyPayload)

    def test_activate(self):
        self.http_client.put.return_value = transport_models.EmptyPayload()

        self.projects.activate(str(t_project.id))

        path, model, body = self.http_client.put.call_args[0]
        self.assertEqual(path, paths_gen.Projects.ACTIVATE_PROJECT.format(project_id=t_project.id))
        self.assertEqual(model, transport_models.EmptyPayload)
        self.assertIsInstance(body, transport_models.EmptyPayload)

    def test_create_program(self):
        self.http_client.post.return_value = base_program.to_transport_model()
        source = BytesIO(b"print('hi')")

        program = self.projects.create_program(str(t_project.id), source, programs.ProgramType.PYTHON312, name='pick')

        call = self.http_client.post.call_args
        self.assertEqual(call[0][0], paths_gen.Projects.CREATE_PROGRAM_IN_PROJECT.format(project_id=t_project.id))
        self.assertEqual(call[0][1], transport_models.s.Program)
        self.assertEqual(call[0][2].name, 'pick')
        self.assertEqual(call[0][2].type, transport_models.s.ProgramType.PYTHON3_12)
        self.assertEqual(call.kwargs['files'], {'file': source})
        self.assertEqual(call.kwargs['exclude'], {'file'})
        self.assertEqual(program, base_program)

    def test_add_program(self):
        self.http_client.put.return_value = transport_models.EmptyPayload()

        self.projects.add_program(str(t_project.id), base_program.id)

        path, model, body = self.http_client.put.call_args[0]
        self.assertEqual(path, paths_gen.Projects.ADD_PROGRAM_TO_PROJECT.format(project_id=t_project.id,
                                                                                program_id=base_program.id))
        self.assertEqual(model, transport_models.EmptyPayload)
        self.assertIsInstance(body, transport_models.EmptyPayload)

    def test_remove_program(self):
        self.http_client.delete.return_value = transport_models.EmptyPayload()

        self.projects.remove_program(str(t_project.id), base_program.id)

        self.http_client.delete.assert_called_once_with(
            paths_gen.Projects.REMOVE_PROGRAM_FROM_PROJECT.format(project_id=t_project.id, program_id=base_program.id),
            transport_models.EmptyPayload)


if __name__ == '__main__':
    unittest.main()
