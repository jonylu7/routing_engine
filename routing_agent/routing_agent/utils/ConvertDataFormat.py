import json
import yaml
def loadJSONFile(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def loadYAMLFile(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def loadJSONFileToStr(file_path):
    data=loadJSONFile(file_path)
    data=json.dumps(data)
    return data

def convertJSONToStr(jsondata:dict):
    return json.dumps(jsondata)

def convertStrToJSON(str:str)->dict:
    return json.loads(str)

def saveJSONAt(jsondata:dict,path:str):
    try:
        with open(path, "w+") as f:
            json.dump(jsondata, f)
    except:
        raise KeyError("Failed to save file")





