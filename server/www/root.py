from flask import Flask
from ..core.lib.maps.osm_to_traffic_graph import
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World!'

@app.route('/create_map')
def get_map():
    return "created"

@app.route('/create_car')
def get_car():
    return "created"