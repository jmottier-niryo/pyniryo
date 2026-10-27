import logging
import re
import threading
from queue import Queue
from typing import Any, Callable, TypeVar, Type, cast
from datetime import datetime, timedelta

from ..exceptions import DataValidationError
from ..models import Metric as MetricModel, Unsubscribe, get_mtype
from .._internal import topics_gen, transport_models, paths_gen
from .._internal.http import HttpClient
from .._internal.mqtt import MqttClient
from . import BaseAPIComponent

logger = logging.getLogger(__name__)

_METRICS_ID_PATTERN = re.compile(r'^[^/+#\x00-\x20\x7f]+$')

T = TypeVar('T', bound=str | int | float | bool | datetime | timedelta)

SystemMetricsCallback = Callable[[transport_models.a.SystemMetrics], None]


class Metric:
    """
    Descriptor class for defining custom metrics that can be monitored.

    This class is used as a descriptor in user-defined classes to create
    metrics that automatically publish updates when their values change.

    :param name: The name of the metric.
    :param init_value: The initial value of the metric.
    :param _type: The Python type of the metric value.
    """

    def __init__(self, name: str, init_value: T, _type: Type[T]) -> None:
        if _type not in [str, int, float, bool, datetime, timedelta]:
            raise TypeError(f'Unsupported type {_type.__name__} for metric "{name}"')

        if not isinstance(init_value, _type):
            raise TypeError(f'Initial value {init_value} does not match type {_type.__name__} for metric "{name}"')

        self.name = name
        self.value = init_value
        self.type = _type
        self.private_name = ''

    def __set_name__(self, owner, name):  # type: ignore
        self.private_name = '_' + name

    def __get__(self, instance, owner):  # type: ignore
        if instance is None:
            return self
        return getattr(instance, self.private_name, self.value)

    def __set__(self, instance, value) -> None:  # type: ignore
        self.value = value
        queue: Queue[Metric] = cast(Queue[Metric], getattr(instance, '_metrics_queue', None))
        if queue is None:
            raise RuntimeError(
                "Unable to publish the metric update. Make sure to register the metrics before using it.")
        queue.put(self)

        setattr(instance, self.private_name, self.value)

    @property
    def tr_value(self) -> str:
        if isinstance(self.value, datetime):
            return self.value.isoformat()
        elif isinstance(self.value, timedelta):
            return str(self.value.total_seconds())
        else:
            return str(self.value)


class Metrics(BaseAPIComponent):
    """
    Metrics component for declaring and publishing custom metrics.

    This component allows you to define custom metrics that can be monitored
    and tracked during robot operations. Metrics are automatically published
    when their values change.
    """

    _metrics_queue: Queue[Metric | None]

    def __init__(self, http_client: HttpClient, mqtt_client: MqttClient, correlation_id: str) -> None:
        super().__init__(http_client, mqtt_client, correlation_id)
        self._topic = self._mqtt_client.format(topics_gen.CustomMetrics.CUSTOM_METRIC, metrics_id=self._correlation_id)
        self._metrics_queue = Queue()
        threading.Thread(target=self._process_metrics_queue, daemon=True).start()

    def _close(self) -> None:
        self._metrics_queue.put(None)
        self._metrics_queue.join()

    def _process_metrics_queue(self) -> None:
        while True:
            metric = self._metrics_queue.get()
            try:
                if metric is None:
                    break
                self._mqtt_client.publish(
                    self._topic,
                    transport_models.a.CustomMetric(name=metric.name,
                                                    value=metric.tr_value,
                                                    m_type=get_mtype(metric.type, a=True)))
                self._http_client.patch(
                    paths_gen.Metrics.UPDATE_CUSTOM_METRICS.format(metrics_id=self._correlation_id),
                    transport_models.EmptyPayload,
                    transport_models.s.CustomMetrics(metrics=[
                        transport_models.s.CustomMetric(name=metric.name,
                                                        value=metric.tr_value,
                                                        m_type=get_mtype(metric.type))
                    ],
                                                     metrics_id=self._correlation_id))
            except Exception as e:
                logger.error(f'Failed to publish metric update: {e}')
            finally:
                self._metrics_queue.task_done()

    def register_metrics(self, inst: object) -> None:
        """
        Register custom metrics from an object instance.

        This method inspects the provided object for Metric descriptors and registers
        them with the system. After registration, any changes to metric values will
        be automatically published.

        :param inst: An object instance containing Metric descriptors.
        :raises DataValidationError: If the correlation id is not one the robot takes as a metrics id.
        """
        if len(self._correlation_id) > 36 or not _METRICS_ID_PATTERN.match(self._correlation_id):
            raise DataValidationError(f'metrics id {self._correlation_id!r} is not a topic level: it must be at most '
                                      f'36 characters, without "/", "+", "#", whitespace or control characters')
        metrics = [m for m in vars(type(inst)).values() if isinstance(m, Metric)]
        tr_metrics = [
            transport_models.s.CustomMetric(name=m.name, value=m.tr_value, m_type=get_mtype(m.type)) for m in metrics
        ]
        self._http_client.post(paths_gen.Metrics.DECLARE_CUSTOM_METRICS,
                               transport_models.EmptyPayload,
                               transport_models.s.CustomMetrics(metrics=tr_metrics, metrics_id=self._correlation_id))

        setattr(inst, '_metrics_queue', self._metrics_queue)

    def on_system_metrics(self, callback: SystemMetricsCallback) -> Unsubscribe:
        """
        Subscribe to the device's system metrics (host resources and Docker container/volume metrics).

        :param callback: The callback function that takes a SystemMetrics parameter.
        """

        def internal_callback(_: str, metrics: transport_models.a.SystemMetrics) -> None:
            callback(metrics)

        return self._mqtt_client.subscribe(topics_gen.System.METRICS, internal_callback,
                                           transport_models.a.SystemMetrics)

    def on_api_metrics(self, callback: Callable[[dict[str, Any]], None]) -> Unsubscribe:
        """
        Subscribe to the metrics the robot publishes about the requests its API serves, one message per request.
        The specification leaves the payload as a bare object, so it is passed as a dict.

        :param callback: The callback function that takes the payload as a dict.
        """

        def internal_callback(_: str, metrics: transport_models.JsonObject) -> None:
            callback(metrics.root)

        return self._mqtt_client.subscribe(topics_gen.Api.METRICS, internal_callback, transport_models.JsonObject)

    def get_metrics(self, metrics_id: str) -> list[MetricModel]:
        """
        Retrieve custom metrics by their metrics ID.

        :param metrics_id: The ID of the metrics to retrieve.
        :return: A list of Metric objects.
        """
        resp = self._http_client.get(paths_gen.Metrics.GET_CUSTOM_METRICS.format(metrics_id=metrics_id),
                                     transport_models.s.CustomMetrics)
        return [MetricModel.from_transport_model(m) for m in resp.metrics]
