import 'package:flutter/material.dart';
import 'package:app_flutter/service/edit.dart';
import 'package:app_flutter/service/delete.dart';

class TaskCard extends StatefulWidget {
  final String taskId;
  final String title;
  final String description;
  final Function() refreshTasks;
  final String token;

  const TaskCard({
    super.key,
    required this.taskId,
    required this.title,
    required this.description,
    required this.refreshTasks,
    required this.token,
  });

  @override
  _TaskCardState createState() => _TaskCardState();
}

class _TaskCardState extends State<TaskCard> {
  late TextEditingController _titleController;
  late TextEditingController _descriptionController;
  bool _isEditing = false;

  @override
  void initState() {
    super.initState();
    _titleController = TextEditingController(text: widget.title);
    _descriptionController = TextEditingController(text: widget.description);
  }

  @override
  void dispose() {
    _titleController.dispose();
    _descriptionController.dispose();
    super.dispose();
  }

   @override
  void didUpdateWidget(TaskCard oldWidget) {
    super.didUpdateWidget(oldWidget);
    _titleController.text = widget.title;
    _descriptionController.text = widget.description;
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _isEditing
                ? TextFormField(
                    controller: _titleController,
                    decoration: const InputDecoration(
                      labelText: 'Title',
                      ),
                  )
                : Text(
                    widget.title,
                    style: const TextStyle(
                      fontSize: 18, 
                      fontWeight: FontWeight.bold,
                      fontFamily: 'Poppins'
                    ),
                  ),
            const SizedBox(height: 8),
            _isEditing
                ? TextFormField(
                    controller: _descriptionController,
                    decoration: const InputDecoration(labelText: 'Description'),
                  )
                : Text(
                  widget.description,
                ),
            const SizedBox(height: 16),
            Row(
              mainAxisAlignment: MainAxisAlignment.end,
              children: [
                if (!_isEditing)
                  IconButton(
                    onPressed: () {
                      setState(() {
                        _isEditing = true;
                      });
                    },
                    icon: const Icon(Icons.edit),
                  ),
                IconButton(
                  onPressed: () async {
                    if (_isEditing) {
                     await edit(widget.taskId, _titleController.text, _descriptionController.text, widget.token);
                      setState(() {
                        _isEditing = false;
                      });
                      await widget.refreshTasks();
                    } else {
                      await delete(widget.taskId, widget.token);
                      await widget.refreshTasks();
                    }
                  },
                  icon: Icon(_isEditing ? Icons.check : Icons.delete),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
