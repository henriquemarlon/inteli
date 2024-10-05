import 'package:http/http.dart' as http;
import 'dart:convert';

Future<void> delete(String id, String token) async {
  const url = "http://10.0.2.2:8000/api/v1/deleteTask";
  
  final response = await http.delete(
    Uri.parse(url),
    headers: <String, String>{
      'Content-Type': 'application/json',
      'Authorization': 'Bearer $token',
    },
    body: jsonEncode({
      'id': id
    }),
  );

  if (response.statusCode == 200) {
  } else {
    throw Exception('Error: ${response.statusCode}');
  }
}
