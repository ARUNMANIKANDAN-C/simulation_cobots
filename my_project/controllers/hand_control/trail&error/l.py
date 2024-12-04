import requests

headers = {'Accept': 'application/json'}

# Corrected URL
r = requests.get('http://127.0.0.1:5000/get_angles', headers=headers)

# Check if the request was successful
if r.status_code == 200:
    print(f"Response: {r.json()}")
else:
    print(f"Failed to retrieve data. Status code: {r.status_code}")
