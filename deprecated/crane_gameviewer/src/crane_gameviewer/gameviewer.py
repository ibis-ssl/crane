# Copyright (c) 2019 SSL-Roots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer

from crane_gameviewer.gameviewer_widget import GameViewerWidget


class GameViewer(Plugin):

    """
    crane_gameviwer plugin's main class. 
    """

    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane,
                        ''PluginContext''
        """
        super(GameViewer, self).__init__(context)
        self.setObjectName('GameViewer')

        self._context = context

        self._widget = GameViewerWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.update)
        self._timer.start(16) # the argument is msec interval

    def shutdown_plugin(self):
        self._timer.stop()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

