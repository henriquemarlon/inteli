import 'package:flutter/material.dart';
import 'package:app_flutter/models/data_model.dart';
import 'package:app_flutter/wigets/card.dart';
import 'package:app_flutter/service/get.dart';
import 'package:app_flutter/pages/new_task.dart';

class UserTasks extends StatefulWidget {
  final String token;

  const UserTasks({
    super.key,
    required this.token,
  });

  @override
  State<UserTasks> createState() => _UserTasksState();
}

class _UserTasksState extends State<UserTasks> {
  late Future<List<DataModel>> futureTasks = get(widget.token);

  Future<void> refreshTasks() async {
    setState(() {
      futureTasks = get(widget.token);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: const Text('Tasks'),
        ),
        bottomNavigationBar: BottomAppBar(
          shape: const CircularNotchedRectangle(),
          child: SizedBox(
            height: 20.0,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: <Widget>[
                IconButton(
                  icon: const Icon(Icons.note_add),
                  onPressed: () {
                    Navigator.push(
                    context,
                    MaterialPageRoute(
                        builder: (context) => NewTask(
                            token: widget.token,
                            refreshTasks: refreshTasks,)));
                  },
                ),
              ],
            ),
          ),
        ),
        body: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Center(
            child: FutureBuilder<List<DataModel>>(
              future: futureTasks,
              builder: (context, snapshot) {
                if (snapshot.hasData) {
                  return ListView.builder(
                    itemCount: snapshot.data!.length,
                    itemBuilder: (context, index) {
                      final task =
                          snapshot.data![snapshot.data!.length - 1 - index];
                      return TaskCard(
                        token: widget.token,
                        taskId: task.id,
                        title: task.title,
                        description: task.description,
                        refreshTasks: refreshTasks,
                      );
                    },
                  );
                }
                else if (snapshot.hasError) {
                  return Text('Nenhuma task registrada');
                }
                return const CircularProgressIndicator();
              },
            ),
          ),
        ));
  }
}
