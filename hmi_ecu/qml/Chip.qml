import QtQuick
import "."

Rectangle {
    id: chip
    property string text: ""
    property bool selected: false
    signal clicked()

    width: Math.max(52, label.width + 22)
    height: 32
    radius: 9
    color: selected ? Theme.text : Theme.chip

    Text {
        id: label
        anchors.centerIn: parent
        text: chip.text
        font.pixelSize: 14
        font.weight: chip.selected ? Font.Bold : Font.Normal
        color: chip.selected ? Theme.inverse : Theme.text
    }
    MouseArea { anchors.fill: parent; onClicked: chip.clicked() }
}
