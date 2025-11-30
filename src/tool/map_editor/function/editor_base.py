# /home/seheonha/git/NGV_DCAS_SW/src/tool/map_editor/function/editor_base.py
from typing import Tuple, Optional
from editor_state import EditorState

class EditorBase:
    def __init__(self, editor_state: EditorState, figure, axes):
        self.editor_state = editor_state
        self.figure = figure
        self.axes = axes

    def OnModeEntered(self):
        pass

    def OnModeExited(self):
        pass

    def OnMousePress(self, event):
        pass

    def OnMouseMove(self, event):
        pass

    def OnMouseRelease(self, event):
        pass

    def OnKeyPress(self, event):
        pass

    def Redraw(self):
        pass

    @staticmethod
    def GetMouseXY(event) -> Optional[Tuple[float, float]]:
        if event.xdata is None or event.ydata is None:
            return None
        return float(event.xdata), float(event.ydata)
