import QtQuick
import "."

Rectangle {
    id: card
    property string title: ""
    property string subtitle: ""
    property alias headerRight: headerRightSlot.data
    default property alias content: body.data
    property int pad: 22

    color: Theme.card
    radius: Theme.radius
    border.color: Theme.border
    border.width: 1

    Row {
        id: header
        visible: card.title !== ""
        x: card.pad
        y: 18
        spacing: 6
        Text {
            text: card.title
            font.pixelSize: 16
            font.weight: Font.Bold
            color: Theme.text
        }
        Text {
            text: card.subtitle
            font.pixelSize: 15
            color: Theme.sub
            anchors.baseline: parent.children[0].baseline
        }
    }

    Item {
        id: headerRightSlot
        anchors.right: parent.right
        anchors.rightMargin: card.pad
        y: 14
        width: childrenRect.width
        height: 28
    }

    Item {
        id: body
        anchors.fill: parent
        anchors.leftMargin: card.pad
        anchors.rightMargin: card.pad
        anchors.topMargin: card.title !== "" ? 54 : card.pad
        anchors.bottomMargin: 16
    }
}
