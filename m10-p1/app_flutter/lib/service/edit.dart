import 'package:http/http.dart' as http;
import 'dart:convert';

Future<void> edit(String id, String newTitle, String newDescription, String token) async {
  const url = "http://10.0.2.2:8000/api/v1/editTask";
  
  final response = await http.put(
    Uri.parse(url),
    headers: <String, String>{
      'Content-Type': 'application/json',
      'Authorization': 'Bearer $token',
    },
    body: jsonEncode({
      'id': id,
      'title': newTitle,
      'description': newDescription,
    }),
  );

  if (response.statusCode == 200) {
  } else {
    throw Exception('Error: ${response.statusCode}');
  }
}
