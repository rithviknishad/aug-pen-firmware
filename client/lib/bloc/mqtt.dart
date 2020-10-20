import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';

class PositionSensor {
  static bool connected;

  final _client = MqttServerClient('', 'client-app')
    ..logging(on: false)
    ..keepAlivePeriod = 20
    ..onConnected = (() => connected = true)
    ..onDisconnected = (() => connected = false)
    ..connectionMessage = (MqttConnectMessage()
        .withClientIdentifier('Mqtt_MyClientUniqueId')
        .keepAliveFor(20) // Must agree with the keep alive set above or not set
        .startClean() // Non persistent session for testing
        .withWillQos(MqttQos.atLeastOnce));

  String get server => _client.server;
  set server(String value) => _client.server = value;

  int get port => _client.port;
  set port(int value) => _client.port = value;

  Future<void> connect() async {
    try {
      await _client.connect();
    } catch (e) {
      print(e);
    }
    if (_client.connectionStatus.state == MqttConnectionState.connected) {
      print('Client connected');

      _client.subscribe('pos/x', MqttQos.atMostOnce);
      _client.subscribe('pos/y', MqttQos.atMostOnce);
      _client.subscribe('pos/z', MqttQos.atMostOnce);
    } else {
      print(
          'Client connection failed - disconnecting, status is ${_client.connectionStatus}');
      _client.disconnect();
    }
  }
}
