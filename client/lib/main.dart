import 'package:flutter/material.dart';
import 'package:position3d/bloc/mqtt.dart';
import 'package:position3d/connect.dart';
import 'package:position3d/interface.dart';

void main() => runApp(MaterialApp(
      title: '3D Position',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: SafeArea(
          child: PositionSensor.instance.connected ? Interface() : Connect()),
    ));
