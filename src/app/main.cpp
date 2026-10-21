#include <QApplication>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTextStream>
#include <QTimer>
#include <cstring>

#include <QQmlContext>

#include "RecordedData.h"
#include "Recorded.h"
#include "ScenarioRunner.h"

namespace {

// Qt's own bundled PNGs carry an old-style colour profile that libpng complains about on every start:
//   "libpng warning: iCCP: known incorrect sRGB profile"
// It is harmless (the images decode fine), so drop that one message and pass everything else through unchanged.
QtMessageHandler g_previousHandler = nullptr;
void filteredMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& message) {
    if (message.contains("iCCP: known incorrect sRGB profile")) return;
    if (g_previousHandler) g_previousHandler(type, context, message);
}

void addScenarioOptions(QCommandLineParser& parser) {
    parser.addOption({"list-scenarios", "Print the available scenarios and their parameters, then exit."});
    parser.addOption({"run-scenario", "Run a scenario headless (no window) and print its report. Exit code 0 = pass, 1 = fail.", "name"});
    parser.addOption({"set", "Override a scenario parameter (repeatable), e.g. --set speed=80.", "key=value"});
    parser.addOption({"repeat", "Run the scenario n times and check that every run gives identical results.", "n", "1"});
    parser.addOption({"report-dir", "Also write <name>.md and <name>.json into this directory.", "dir"});
    parser.addOption({"json", "Print the report as JSON instead of Markdown."});
    parser.addOption({"recorded-info", "Load a KITTI raw drive folder, print what it contains (frames, duration, sensors, warnings) and exit.", "dir"});
}

// Headless scenario runner for scripts and CI. Same deterministic simulation as the GUI.
int runCli(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    QTextStream out(stdout), err(stderr);
    QCommandLineParser parser;
    parser.setApplicationDescription("DriveForge headless scenario runner (educational simulator, not safety-certified)");
    parser.addHelpOption();
    addScenarioOptions(parser);
    parser.process(app);

    if (parser.isSet("recorded-info")) {
        driveforge::RecordedDataset ds;
        std::string e;
        if (!driveforge::loadKittiRaw(parser.value("recorded-info").toStdString(), &ds, &e)) {
            err << "Cannot load recorded data: " << QString::fromStdString(e) << "\n";
            return 1;
        }
        out << QString::fromStdString(ds.summary()) << "\n";
        out << "source: " << QString::fromStdString(ds.source) << "\nlicense: " << QString::fromStdString(ds.license) << "\n";
        for (const std::string& w : ds.warnings) out << "note: " << QString::fromStdString(w) << "\n";
        return 0;
    }

    if (parser.isSet("list-scenarios")) {
        for (const driveforge::ScenarioInfo& info : driveforge::scenarioCatalog()) {
            out << info.key.c_str() << ": " << info.name.c_str() << "\n    " << info.description.c_str() << "\n";
            for (const driveforge::ParamSpec& p : info.params)
                out << "    " << p.key.c_str() << " = " << p.def << " " << p.unit.c_str() << "  [" << p.min << " .. " << p.max << "]  " << p.label.c_str() << "\n";
        }
        return 0;
    }

    driveforge::ScenarioId id;
    if (!driveforge::scenarioIdFromKey(parser.value("run-scenario").toStdString(), &id)) {
        err << "Unknown scenario '" << parser.value("run-scenario") << "'. Use --list-scenarios.\n";
        return 2;
    }
    driveforge::ParamMap overrides;
    for (const QString& kv : parser.values("set")) {
        const int eq = kv.indexOf('=');
        bool ok = false;
        const double v = eq > 0 ? kv.mid(eq + 1).toDouble(&ok) : 0.0;
        const std::string key = kv.left(eq).toStdString();
        bool known = false;
        for (const driveforge::ParamSpec& p : driveforge::scenarioInfo(id).params) known = known || p.key == key;
        if (!ok || !known) {
            err << "Bad --set '" << kv << "' (expected key=value with a parameter of this scenario).\n";
            return 2;
        }
        overrides[key] = v;
    }
    bool okRepeat = false;
    const int repeat = parser.value("repeat").toInt(&okRepeat);
    if (!okRepeat || repeat < 1 || repeat > 20) {
        err << "--repeat must be between 1 and 20.\n";
        return 2;
    }

    std::vector<driveforge::ScenarioReport> runs;
    for (int i = 0; i < repeat; ++i) runs.push_back(driveforge::ScenarioRunner::runHeadless(id, overrides));
    const driveforge::ScenarioReport& report = runs.front();
    out << (parser.isSet("json") ? report.toJson() : report.toMarkdown()).c_str();

    bool identical = true, allPass = true;
    for (const auto& r : runs) {
        identical = identical && r.sameResultAs(report);
        allPass = allPass && r.passed();
    }
    if (repeat > 1)
        out << "\nRepeatability: " << repeat << " runs, " << (identical ? "all identical" : "RESULTS DIFFERED") << "\n";

    if (parser.isSet("report-dir")) {
        QDir dir(parser.value("report-dir"));
        if (!dir.mkpath(".")) { err << "Cannot create " << dir.path() << "\n"; return 2; }
        const QString base = dir.filePath(QString::fromStdString(driveforge::scenarioInfo(id).key));
        QFile md(base + ".md"), js(base + ".json");
        if (!md.open(QIODevice::WriteOnly | QIODevice::Text) || !js.open(QIODevice::WriteOnly | QIODevice::Text)) {
            err << "Cannot write reports to " << dir.path() << "\n";
            return 2;
        }
        md.write(QByteArray::fromStdString(report.toMarkdown()));
        js.write(QByteArray::fromStdString(report.toJson()));
    }
    return allPass && identical ? 0 : 1;
}

bool wantsCli(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i)
        if (!std::strcmp(argv[i], "--list-scenarios") || !std::strncmp(argv[i], "--run-scenario", 14) || !std::strncmp(argv[i], "--recorded-info", 15)) return true;
    return false;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (wantsCli(argc, argv)) return runCli(argc, argv);

    g_previousHandler = qInstallMessageHandler(filteredMessageHandler);
    QApplication app(argc, argv);  // Qt Charts' QML ChartView requires QApplication
    app.setApplicationName("DriveForge");
    QQuickStyle::setStyle("Fusion");

    QCommandLineParser parser;
    parser.setApplicationDescription("Educational 3D autonomous car simulator (not a safety-certified system)");
    parser.addHelpOption();
    QCommandLineOption shot("screenshot", "Save a window screenshot to <file> and exit (for CI/debugging).", "file");
    QCommandLineOption shotDelay("screenshot-delay", "Delay in ms before the screenshot (default 2000).", "ms", "2000");
    parser.addOption(shot);
    parser.addOption(shotDelay);
    QCommandLineOption cameraOpt("camera", "Camera preset for the 3D view: chase (default), front, rear, side, top, closeup.", "preset", "chase");
    parser.addOption(cameraOpt);
    QCommandLineOption qualityOpt("quality", "Graphics quality: low, medium or high (default). F4 cycles it in the app.", "level", "high");
    parser.addOption(qualityOpt);
    QCommandLineOption recordedOpt("recorded", "Open the recorded-data view with this KITTI drive folder loaded.", "dir");
    parser.addOption(recordedOpt);
    addScenarioOptions(parser);  // listed in --help; handled by the headless runner when used
    parser.process(app);

    QQmlApplicationEngine engine;
    engine.addImageProvider("recorded", new RecordedImageProvider);  // engine takes ownership
    // Where the README says to put the downloaded KITTI drive: data/kitti in the project. Search upwards from the executable
    // (works from build/, from Qt Creator's build/Desktop_Qt_...-Release/, and from the current folder).
    QString defaultDir;
    {
        QDir dir(QCoreApplication::applicationDirPath());
        for (int i = 0; i < 6 && defaultDir.isEmpty(); ++i) {
            if (QDir(dir.filePath("data/kitti")).exists()) defaultDir = QDir(dir.filePath("data/kitti")).absolutePath();
            if (!dir.cdUp()) break;
        }
        if (defaultDir.isEmpty() && QDir(QDir::currentPath() + "/data/kitti").exists()) defaultDir = QDir::currentPath() + "/data/kitti";
    }
    engine.rootContext()->setContextProperty("startupRecordedDir", parser.value(recordedOpt));
    engine.rootContext()->setContextProperty("startupCamera", parser.value(cameraOpt));
    {
        const QString level = parser.value(qualityOpt).toLower();
        engine.rootContext()->setContextProperty("startupQuality", level == "low" ? 0 : level == "medium" ? 1 : 2);
    }
    engine.rootContext()->setContextProperty("defaultRecordedDir", defaultDir);
    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(1); },
        Qt::QueuedConnection);
    engine.loadFromModule("DriveForge", "Main");

    if (parser.isSet(shot)) {
        const QString path = parser.value(shot);
        QTimer::singleShot(parser.value(shotDelay).toInt(), &app, [&engine, path] {
            auto* window = qobject_cast<QQuickWindow*>(engine.rootObjects().value(0));
            const bool ok = window && window->grabWindow().save(path);
            QCoreApplication::exit(ok ? 0 : 2);
        });
    }
    return app.exec();
}
