#pragma once

#include <map>
#include <string>
#include <vector>

namespace driveforge {

enum class ScenarioId { Highway = 0, Obstacle, Pedestrian, Parking };

using ParamMap = std::map<std::string, double>;

struct ParamSpec {
    std::string key, label, unit;
    double def = 0.0, min = 0.0, max = 1.0, step = 1.0;
};

struct ScenarioInfo {
    ScenarioId id;
    std::string key;  // used on the command line
    std::string name;
    std::string description;
    std::vector<ParamSpec> params;
};

const std::vector<ScenarioInfo>& scenarioCatalog();
const ScenarioInfo& scenarioInfo(ScenarioId id);
bool scenarioIdFromKey(const std::string& key, ScenarioId* id);

ParamMap defaultParams(ScenarioId id);
// Defaults, overridden by `overrides`, each clamped to its [min, max]. Unknown keys are ignored.
ParamMap resolveParams(ScenarioId id, const ParamMap& overrides);

struct Criterion {
    std::string name;
    std::string requirement;
    std::string measured;
    bool passed = false;
};

enum class Outcome { Pass, Fail, Aborted };

struct ScenarioReport {
    ScenarioId id = ScenarioId::Highway;
    std::string name;
    ParamMap params;
    Outcome outcome = Outcome::Aborted;
    double duration = 0.0;  // simulated seconds
    std::string endReason;
    std::vector<Criterion> criteria;
    std::map<std::string, double> metrics;  // everything measured; compared for repeatability
    std::vector<std::string> notes;

    bool passed() const { return outcome == Outcome::Pass; }
    std::string outcomeText() const;
    std::string toMarkdown() const;
    std::string toJson() const;
    // True when both runs measured exactly the same values (same outcome, duration and metrics).
    bool sameResultAs(const ScenarioReport& other) const;
};

}  // namespace driveforge
