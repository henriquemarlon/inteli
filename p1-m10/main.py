from flask import Flask, request, jsonify
import psycopg2

def get_db_connection():
    conn = psycopg2.connect(host='host.docker.internal',
                            database='db',
                            user='user',
                            password='password')
    return conn

app = Flask(__name__)

@app.route('/novo', methods=['POST'])
def addOrder():
    conn = get_db_connection()
    cur = conn.cursor()
    data = request.get_json()
    name = data['name']
    email = data['email']
    description = data['description']
    cur.execute("INSERT INTO orders (name, email, description) VALUES (%s, %s, %s) RETURNING id", 
                (name, email, description))
    conn.commit()
    inserted_id = cur.fetchone()[0]
    cur.close()
    conn.close()
    if not inserted_id:
        return jsonify({"message": "Order not created"}), 500
    return jsonify({"id": inserted_id})

@app.route('/pedidos', methods=['GET'])
def getOrders():
    conn = get_db_connection()
    cur = conn.cursor()
    cur.execute("SELECT id, name, email, description FROM orders")
    rows = cur.fetchall()
    cur.close()
    conn.close()
    
    orders = []
    for row in rows:
        order = {
            "id": row[0],
            "name": row[1],
            "email": row[2],
            "description": row[3]
        }
        orders.append(order)
    
    return jsonify(orders)

@app.route('/pedidos/<string:id>', methods=['GET'])
def getOrder(id):
    conn = get_db_connection()
    cur = conn.cursor()
    cur.execute("SELECT id, name, email, description FROM orders WHERE id = %s", (id,))
    row = cur.fetchone()
    cur.close()
    conn.close()
    
    if not row:
        return jsonify({"message": "Order not found"}), 404
    
    order = {
        "id": row[0],
        "name": row[1],
        "email": row[2],
        "description": row[3]
    }
    
    return jsonify(order)

@app.route('/pedidos/<string:id>', methods=['DELETE'])
def deleteOrder(id):
    conn = get_db_connection()
    cur = conn.cursor()
    cur.execute("DELETE FROM orders WHERE id = %s", (id,))
    conn.commit()
    cur.close()
    conn.close()
    return jsonify({"message": "Order deleted"})

@app.route('/pedidos/<string:id>', methods=['PUT'])
def updateOrder(id):
    conn = get_db_connection()
    cur = conn.cursor()
    data = request.get_json()
    name = data['name']
    email = data['email']
    description = data['description']
    cur.execute("UPDATE orders SET name = %s, email = %s, description = %s WHERE id = %s RETURNING id, name, email, description", 
                (name, email, description, id))
    conn.commit()
    row = cur.fetchone()
    cur.close()
    conn.close()
    
    if not row:
        return jsonify({"message": "Order not found"}), 404
    
    order = {
        "id": row[0],
        "name": row[1],
        "email": row[2],
        "description": row[3]
    }
    return jsonify(order)

if __name__ == "__main__":
    app.run(debug=True)
