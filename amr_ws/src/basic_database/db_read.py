import sqlite3
from faker import Faker

# sudo apt-get install sqlite3
# sqlite3 contacts.db (Create a SQLite Database and Table)
# Inside the db -->
# CREATE TABLE contact_info (
#     id INTEGER PRIMARY KEY AUTOINCREMENT,
#     name TEXT NOT NULL,
#     phone_number TEXT NOT NULL
# );
# .quit

# CREATE TABLE node_job (
#     id INTEGER PRIMARY KEY AUTOINCREMENT,
#     Node TEXT NOT NULL,
#     Job TEXT NOT NULL
# );

# Connect to the SQLite database
conn = sqlite3.connect('contacts.db')
cursor = conn.cursor()
cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")

def create_contact(name, phone_number):
    cursor.execute("INSERT INTO contact_info (name, phone_number) VALUES (?, ?)", (name, phone_number))
    conn.commit()
    print(f"Contact '{name}' with phone number '{phone_number}' added successfully.")

def delete_row_by_id(row_id):
    try:
        # Execute a DELETE query targeting the specified row ID
        cursor.execute(f"SELECT * FROM contact_info WHERE rowid = {row_id};")
        last_row_id = cursor.fetchone()[0]
        print(last_row_id)
        cursor.execute(f"DELETE FROM contact_info WHERE rowid = {row_id};")
        conn.commit()
        print(f"Row with ID {row_id} deleted successfully.")
    except sqlite3.Error as e:
        print(f"Error deleting row: {e}")
    finally:
        # Close the connection
        conn.close()

def clear_db():
    cursor.execute("SELECT MAX(rowid) FROM contact_info")
    last_row_id = cursor.fetchone()[0]
    while last_row_id is not None:
        cursor.execute(f"DELETE FROM contact_info WHERE rowid = {last_row_id}")
        conn.commit()
        cursor.execute("SELECT MAX(rowid) FROM contact_info")
        last_row_id = cursor.fetchone()[0]
        if last_row_id is None:
            print("Job done")

def create_random_names_no():
    fake = Faker()
    for _ in range(10):
        random_name = fake.name()
        random_phone_number = fake.phone_number()
        create_contact(random_name, random_phone_number)

# sqlite3 contact_info.db "UPDATE contact_info SET name='New Name', phone_number='New Phone Number' WHERE rowid=2;"
def update_contact(row_id, new_name, new_phone_number):
    try:
        # Execute an UPDATE query to update the row with the specified rowid
        cursor.execute(f"UPDATE contact_info SET name=?, phone_number=? WHERE rowid=?;", (new_name, new_phone_number, row_id))
        conn.commit()
        print(f"Updated row with rowid {row_id}")

    except sqlite3.Error as e:
        print(f"Error updating row: {e}")
    finally:
        # Close the connection
        conn.close()




create_random_names_no()
# update_contact(5, 'debanik', '2556')
# delete_row_by_id(1)
# clear_db()
# Example usage
# create_contact("John Doe", "123-456-7890")
# create_contact("Roy", "974543535")

# table_names = cursor.fetchall()
# print(table_names[0][0])
# table_name = table_names[0][0]

# cursor.execute("SELECT MAX(rowid) FROM contact_info")
# last_row_id = cursor.fetchone()[0]

# while last_row_id is not None:
#     print(last_row_id)
#     cursor.execute(f"DELETE FROM contact_info WHERE rowid = {last_row_id}")
#     conn.commit()
#     cursor.execute("SELECT MAX(rowid) FROM contact_info")
#     last_row_id = cursor.fetchone()[0]
# delete_row_by_id(0)
# Close the connection
conn.close()