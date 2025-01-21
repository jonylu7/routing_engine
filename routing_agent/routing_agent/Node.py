from Vector import Vector3
class Node:
    def __init__(self,id:str="DEFAULT_NODE",locallocation:Vector3=Vector3(),mapid:str="DEFAULT_MAP",edges:list[str]=[],isentrypoint:bool=False,entrytonodeid:str="DEFAULT_ENTRY_TO_NODE"):
        self.id=id
        self.edges=edges
        self.localLocation=locallocation
        self.mapId=mapid
        self.isEntryPoint=isentrypoint
        self.entryToNodeId=entrytonodeid
    
    def setEntryPointById(self,nodeid:str):
        self.isEntryPoint=True
        self.entryToNodeId=nodeid
