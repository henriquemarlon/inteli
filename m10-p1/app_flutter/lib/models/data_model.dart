import 'dart:convert';

List<DataModel> taskFromJson(String str) {
  final Iterable<dynamic> parsed = json.decode(str);
  return List<DataModel>.from(parsed.map((x) => DataModel.fromJson(x)));
}
String taskToJson(List<DataModel> data) => json.encode(List<dynamic>.from(data.map((x) => x.toJson())));
class DataModel {
    String id;
    String userId;
    String title;
    String description;

    DataModel({
        required this.id,
        required this.userId,
        required this.title,
        required this.description,
    });

    factory DataModel.fromJson(Map<String, dynamic> json) => DataModel(
        id: json["id"],
        userId: json["user_id"],
        title: json["title"],
        description: json["description"],
    );

    Map<String, dynamic> toJson() => {
        "id": id,
        "user_id": userId,
        "title": title,
        "description": description,
    };
}
