import sqlite3
con = sqlite3.connect("sqlite.db")
cur = con.cursor()

cur.execute("CREATE TABLE tasks (title TEXT, content TEXT)")
cur.execute("CREATE TABLE users (username TEXT, password TEXT)")

con.commit()
con.close()