import 'package:flutter/material.dart';
import 'package:app_flutter/pages/user_tasks.dart';

class Login extends StatelessWidget {
  const Login({super.key});

  @override
  Widget build(context) {
    return Scaffold(
        backgroundColor: Colors.transparent,
        body: Center(
            child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text('Login',
                style: TextStyle(
                    color: Colors.white,
                    fontSize: 32,
                    fontWeight: FontWeight.bold,
                    fontFamily: 'Montserrat')),
            const Padding(
                padding: EdgeInsetsDirectional.only(
                    start: 64.0, end: 64.0, top: 44.0),
                child: TextField(
                  decoration: InputDecoration(
                    border: OutlineInputBorder(
                      borderSide: BorderSide(color: Colors.white),
                    ),
                    hintText: 'Email',
                    hintStyle: TextStyle(
                        color: Colors.white, fontFamily: 'Montserrat'),
                  ),
                )),
            const Padding(
                padding: EdgeInsetsDirectional.only(
                    start: 64.0, end: 64.0, bottom: 44.0, top: 16.0),
                child: TextField(
                  style: TextStyle(color: Colors.white),
                  decoration: InputDecoration(
                    border: OutlineInputBorder(
                      borderSide: BorderSide(color: Colors.white),
                    ),
                    hintText: 'Password',
                    hintStyle: TextStyle(
                        color: Colors.white, fontFamily: 'Montserrat'),
                  ),
                )),
            ElevatedButton(
              onPressed: () {
                Navigator.push(
                    context,
                    MaterialPageRoute(
                        builder: (context) => const UserTasks(
                            token:
                                'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJlbWFpbCI6ImhlbnJpcXVlbWFybG9uQGdtYWlsLmNvbSIsImV4cCI6MTcxNTkzNTUxM30._ZW6vV7KDyTDP7zWSi3oTeECKhUcbTPR473Z7LAfpRY')));
              },
              child: const Text('Login',
                  style: TextStyle(
                      color: Colors.blueAccent,
                      fontSize: 18,
                      fontWeight: FontWeight.bold,
                      fontFamily: 'Montserrat')),
            ),
          ],
        )));
  }
}
