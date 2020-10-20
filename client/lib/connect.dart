import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

class Connect extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    SystemChrome.setSystemUIOverlayStyle(SystemUiOverlayStyle(
      statusBarColor: Color.fromARGB(255, 199, 203, 152),
      statusBarIconBrightness: Brightness.dark,
    ));
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Color.fromARGB(255, 199, 203, 152),
        title: Text(
          '3D Position',
          style: TextStyle(color: Colors.black),
        ),
        elevation: 0,
      ),
      body: Container(
        child: Text('connect'),
      ),
    );
  }
}
