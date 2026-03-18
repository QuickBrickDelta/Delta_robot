import QtQuick
import QtQuick.Window
import QtQuick.Controls

Window {
    id: root
    visible: true
    visibility: Window.FullScreen
    title: qsTr("Hello World")

    Shortcut {
        sequence: "Esc"
        onActivated: Qt.quit()
    }
}
