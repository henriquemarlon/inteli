extends KinematicBody2D

func _ready():
	$HTTPRequest.request('http://127.0.0.1:5000/coordinates/last')
	

func _on_Timer_timeout():
	$HTTPRequest.request('http://127.0.0.1:5000/coordinates/last')

	
func _on_HTTPRequest_request_completed(result, response_code, headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var x = json.result["x"]
	var y = json.result["y"]
	var z = json.result["z"]
	var r = json.result["r"]
	move_robot(x, y, z, r)
	
var tween = null
func move_robot(x, y, z, r):
	tween = get_tree().create_tween()
	var xy  = Vector2(x, y)
	defineXY(xy)
	defineZ(z)
	defineR(r)
	
func defineXY(atual):
	var rotation = ( atual - global_position).angle()
	tween.tween_property(self, "rotation", rotation, 2)

func defineZ(z):
	var alvo  = get_node("alvo")
	z= clamp(z, 0, 100)
	z = 0.1  + z/100.0
	tween.tween_property(alvo, "scale", Vector2(z, z), 1)

func defineR(r):
	var rotation = get_node("alvo")
	tween.tween_property(rotation, "rotation_degrees", clamp(r, 0, 360), 1)
	

	
	



	
	
