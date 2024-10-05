import 'package:app_flutter/models/data_model.dart';
import 'package:http/http.dart' as http;

Future<List<DataModel>> get(String token) async {
  const url = "http://10.0.2.2:8000/api/v1/getTasks/1";

  final response = await http.get(
    Uri.parse(url),
    headers: <String, String>{
      'Content-Type': 'application/json',
      'Authorization': 'Bearer $token',
    },);

  if (response.statusCode == 200) {
    return taskFromJson(response.body);
  } else {
    throw Exception("Error");
  }
}