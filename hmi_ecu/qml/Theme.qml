pragma Singleton
import QtQuick

QtObject {
    // 야간 모드 (헤더 버튼으로 전환). 밤 주행 시 눈부심 감소
    property bool dark: false

    // -------- 화면 바탕 --------
    readonly property color bg: dark ? "#0D1116" : "#EDF0F4"
    readonly property color panel: dark ? "#12171E" : "#F6F8FA"
    readonly property color card: dark ? "#171D25" : "#FFFFFF"
    readonly property color cardSoft: dark ? "#1E252E" : "#F5F7FA"
    readonly property color chip: dark ? "#252D37" : "#EEF1F5"
    readonly property color border: dark ? "#29313C" : "#E3E7ED"
    readonly property color track: dark ? "#2A323D" : "#E7EAEF"
    readonly property color platform: dark ? "#1F262F" : "#E6EAEF"
    readonly property color platformEdge: dark ? "#2C3540" : "#D9DEE5"
    readonly property color ringOuter: dark ? "#1F262F" : "#EEF0F3"

    // -------- 글자 --------
    readonly property color text: dark ? "#E9EDF2" : "#1A1E24"
    readonly property color sub: dark ? "#9BA4AF" : "#69717D"
    readonly property color faint: dark ? "#5E6772" : "#A3AAB4"
    readonly property color inverse: dark ? "#0D1116" : "#FFFFFF"

    // -------- 도로 장면 --------
    readonly property color sky1: dark ? "#10151B" : "#FAFBFC"
    readonly property color sky2: dark ? "#141A21" : "#EEF1F4"
    readonly property color sky3: dark ? "#1A2129" : "#E3E7EC"
    readonly property color road1: dark ? "#1C232B" : "#E6E9ED"
    readonly property color road2: dark ? "#262E38" : "#CDD3DA"
    readonly property color laneLine: dark ? "#6B7683" : "#FFFFFF"

    // -------- 상태 색 --------
    readonly property color blue: "#2F7BEB"
    readonly property color green: "#1FB257"
    readonly property color greenSoft: dark ? "#16351F" : "#DDF5E6"
    readonly property color yellow: "#F2B705"
    readonly property color orange: "#FF8A1F"
    readonly property color red: "#EE2B2B"
    readonly property color redSoft: dark ? "#3A1717" : "#FDE2E2"
    readonly property color gray: "#8F97A2"

    readonly property int radius: 18

    // -------- helpers --------
    function has(v) { return v !== undefined && v !== null }

    function fmt(v, digits, fallback) {
        if (!has(v)) return fallback === undefined ? "--" : fallback
        return Number(v).toFixed(digits === undefined ? 0 : digits)
    }

    function clamp01(v) {
        if (!has(v)) return 0
        return Math.max(0, Math.min(1, v))
    }

    function levelColor(level) {
        if (!has(level)) return gray
        switch (level) {
        case 0: return green
        case 1: return yellow
        case 2: return orange
        case 3: return red
        }
        return gray
    }

    function severityColor(sev) {
        switch (sev) {
        case "ok": return green
        case "caution": return yellow
        case "warning": return orange
        case "danger": return red
        case "fault": return red
        case "info": return blue
        }
        return gray
    }

    function severitySoft(sev) {
        var c = severityColor(sev)
        if (sev === "offline" || c === gray) return dark ? "#1E252E" : "#EEF0F3"
        return Qt.rgba(c.r, c.g, c.b, dark ? 0.2 : 0.14)
    }

    // 연결 상태 3단계: true / false / null(확인 불가)
    function linkColor(v) { return v === true ? green : (v === false ? red : gray) }
    function linkText(v) { return v === true ? "Connected" : (v === false ? "Disconnected" : "No Data") }
}
