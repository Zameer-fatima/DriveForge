#include "ScenarioManager.h"

#include <QDir>
#include <QFile>
#include <QStandardPaths>
#include <QVariantMap>

using namespace driveforge;

// ---- ScenarioParamModel -------------------------------------------------------------------------------------------

int ScenarioParamModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : static_cast<int>(scenarioInfo(id_).params.size());
}

QVariant ScenarioParamModel::data(const QModelIndex& index, int role) const {
    const auto& specs = scenarioInfo(id_).params;
    if (!index.isValid() || index.row() < 0 || index.row() >= static_cast<int>(specs.size()) || !values_) return {};
    const ParamSpec& p = specs[static_cast<std::size_t>(index.row())];
    switch (role) {
    case KeyRole: return QString::fromStdString(p.key);
    case LabelRole: return QString::fromStdString(p.label);
    case UnitRole: return QString::fromStdString(p.unit);
    case ValueRole: return values_->at(p.key);
    case MinRole: return p.min;
    case MaxRole: return p.max;
    case StepRole: return p.step;
    }
    return {};
}

QHash<int, QByteArray> ScenarioParamModel::roleNames() const {
    return {{KeyRole, "key"}, {LabelRole, "label"}, {UnitRole, "unit"}, {ValueRole, "value"},
            {MinRole, "from"}, {MaxRole, "to"},      {StepRole, "step"}};
}

void ScenarioParamModel::show(ScenarioId id, ParamMap* values) {
    beginResetModel();
    id_ = id;
    values_ = values;
    endResetModel();
}

void ScenarioParamModel::refresh() {
    const int n = rowCount();
    if (n > 0) emit dataChanged(index(0), index(n - 1), {ValueRole});
}

bool ScenarioParamModel::set(int row, double value) {
    const auto& specs = scenarioInfo(id_).params;
    if (!values_ || row < 0 || row >= static_cast<int>(specs.size())) return false;
    const ParamSpec& p = specs[static_cast<std::size_t>(row)];
    value = std::min(std::max(value, p.min), p.max);
    (*values_)[p.key] = value;
    emit dataChanged(index(row), index(row), {ValueRole});
    return true;
}

// ---- ScenarioManager ----------------------------------------------------------------------------------------------

ScenarioManager::ScenarioManager(Simulation& sim, QObject* parent) : QObject(parent), sim_(sim) {
    for (const ScenarioInfo& info : scenarioCatalog()) params_[info.id] = defaultParams(info.id);
    paramModel_.show(selected_, &params_[selected_]);
}

QVariantList ScenarioManager::catalog() const {
    QVariantList list;
    for (const ScenarioInfo& info : scenarioCatalog()) {
        QVariantMap m;
        m["name"] = QString::fromStdString(info.name);
        m["key"] = QString::fromStdString(info.key);
        list.append(m);
    }
    return list;
}

void ScenarioManager::setSelected(int index) {
    const auto& catalog = scenarioCatalog();
    if (index < 0 || index >= static_cast<int>(catalog.size()) || catalog[static_cast<std::size_t>(index)].id == selected_) return;
    selected_ = catalog[static_cast<std::size_t>(index)].id;
    paramModel_.show(selected_, &params_[selected_]);
    repeatSummary_.clear();
    emit selectedChanged();
    emit reportChanged();
}

QString ScenarioManager::description() const { return QString::fromStdString(scenarioInfo(selected_).description); }

QString ScenarioManager::status() const {
    if (runner_.running() || runner_.finished()) return QString::fromStdString(runner_.statusText());
    return QStringLiteral("no scenario running");
}

void ScenarioManager::setTimeScale(double scale) {
    scale = std::min(std::max(scale, 1.0), 8.0);
    if (scale == timeScale_) return;
    timeScale_ = scale;
    emit timeScaleChanged();
}

void ScenarioManager::setParam(int row, double value) {
    if (paramModel_.set(row, value)) repeatSummary_.clear();
}

void ScenarioManager::resetParams() {
    params_[selected_] = defaultParams(selected_);
    paramModel_.refresh();
}

void ScenarioManager::runLive() {
    if (runner_.running()) runner_.abort(sim_, "restarted");
    runner_.start(sim_, selected_, params_[selected_]);
    hasReport_ = false;
    repeatSummary_.clear();
    exportPath_.clear();
    wasRunning_ = true;
    emit runningChanged();
    emit statusChanged();
    emit reportChanged();
    emit liveStarted(QString::fromStdString(scenarioInfo(selected_).name));
}

void ScenarioManager::abort(const QString& reason) {
    if (!runner_.running()) return;
    runner_.abort(sim_, reason.toStdString());
    afterStep();
}

void ScenarioManager::afterStep() {
    runner_.afterStep(sim_);
    if (wasRunning_ && !runner_.running() && runner_.finished()) {
        wasRunning_ = false;
        takeReport(runner_.report());
        emit runningChanged();
        const ScenarioReport& r = runner_.report();
        emit finished(QString::fromStdString(r.name), QString::fromStdString(r.outcomeText()), QString::fromStdString(r.endReason));
    }
    if (runner_.running()) emit statusChanged();
}

void ScenarioManager::takeReport(const ScenarioReport& report) {
    report_ = report;
    hasReport_ = true;
    reportTime_ = QDateTime::currentDateTime();
    exportPath_.clear();
    emit statusChanged();
    emit reportChanged();
}

void ScenarioManager::runRepeatCheck(int times) {
    if (runner_.running()) return;
    times = std::min(std::max(times, 2), 10);
    std::vector<ScenarioReport> runs;
    for (int i = 0; i < times; ++i) runs.push_back(ScenarioRunner::runHeadless(selected_, params_[selected_]));
    bool identical = true;
    for (const ScenarioReport& r : runs) identical = identical && r.sameResultAs(runs.front());
    int passes = 0;
    for (const ScenarioReport& r : runs) passes += r.passed() ? 1 : 0;
    takeReport(runs.front());
    repeatSummary_ = QString("Repeated %1 times: %2 (%3 of %1 passed)")
                         .arg(times)
                         .arg(identical ? "all runs gave identical results" : "RESULTS DIFFERED")
                         .arg(passes);
    emit reportChanged();
}

QString ScenarioManager::summary() const {
    if (!hasReport_) return {};
    int ok = 0;
    for (const Criterion& c : report_.criteria) ok += c.passed ? 1 : 0;
    QString s = QString("%1 of %2 criteria met · %3 s simulated · ended: %4")
                    .arg(ok).arg(report_.criteria.size()).arg(report_.duration, 0, 'f', 1)
                    .arg(QString::fromStdString(report_.endReason));
    if (report_.outcome == Outcome::Aborted) s = QString("aborted after %1 s · %2").arg(report_.duration, 0, 'f', 1).arg(QString::fromStdString(report_.endReason));
    return s;
}

QVariantList ScenarioManager::criteria() const {
    QVariantList list;
    for (const Criterion& c : report_.criteria) {
        QVariantMap m;
        m["name"] = QString::fromStdString(c.name);
        m["requirement"] = QString::fromStdString(c.requirement);
        m["measured"] = QString::fromStdString(c.measured);
        m["passed"] = c.passed;
        list.append(m);
    }
    return list;
}

QString ScenarioManager::exportReport() {
    if (!hasReport_) return {};
    const QString dirPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/reports";
    if (!QDir().mkpath(dirPath)) return {};
    const QString base = dirPath + "/" + QString::fromStdString(scenarioInfo(report_.id).key) + "-" +
                         reportTime_.toString("yyyyMMdd-HHmmss");
    QFile md(base + ".md"), js(base + ".json");
    if (!md.open(QIODevice::WriteOnly | QIODevice::Text) || !js.open(QIODevice::WriteOnly | QIODevice::Text)) return {};
    md.write(QByteArray::fromStdString(report_.toMarkdown()));
    md.write(QString("\nGenerated: %1 (wall clock)\n").arg(reportTime_.toString(Qt::ISODate)).toUtf8());
    if (!repeatSummary_.isEmpty()) md.write(("\nRepeatability: " + repeatSummary_ + "\n").toUtf8());
    js.write(QByteArray::fromStdString(report_.toJson()));
    exportPath_ = base + ".md (+ .json)";
    emit reportChanged();
    return exportPath_;
}
