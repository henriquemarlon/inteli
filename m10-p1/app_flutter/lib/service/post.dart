import 'package:http/http.dart' as http;
import 'dart:convert';

Future<void> post(String title, String description, String token) async {
  const url = "http://10.0.2.2:8000/api/v1/createTask/1";
  
  final response = await http.post(
    Uri.parse(url),
    headers: <String, String>{
      'Content-Type': 'application/json',
      'Authorization': 'Bearer $token',
    },
    body: jsonEncode({
      'title': title,
      'description': description
    }),
  );

  if (response.statusCode == 200) {
  } else {
    throw Exception('Error: ${response.statusCode}');
  }
}
