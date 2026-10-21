#include "Scenario.h"

#include <algorithm>
#include <cstdio>

namespace driveforge {

namespace {
std::string num(double v, int decimals = 2) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    return buf;
}

std::string jsonString(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c == '\n') out += "\\n";
        else out += c;
    }
    return out + "\"";
}

std::vector<ScenarioInfo> buildCatalog() {
    std::vector<ScenarioInfo> c;
    c.push_back({ScenarioId::Highway, "highway", "Highway following",
                 "Autonomous mode (ACC + lane keeping + AEB) follows a slower lead car that then brakes.",
                 {{"egoSpeed", "Ego start speed", "km/h", 100, 60, 130, 5},
                  {"accSpeed", "ACC set speed", "km/h", 110, 60, 130, 5},
                  {"timeGap", "ACC time gap", "s", 1.8, 1.0, 3.0, 0.2},
                  {"leadSpeed", "Lead car speed", "km/h", 70, 30, 110, 5},
                  {"gap", "Initial gap", "m", 120, 40, 200, 10},
                  {"leadBrakeAt", "Lead brakes at", "s", 15, 5, 40, 1},
                  {"leadDecel", "Lead deceleration", "m/s²", 3.0, 1.0, 7.0, 0.5},
                  {"leadFinalSpeed", "Lead final speed", "km/h", 30, 0, 80, 5},
                  {"duration", "Duration", "s", 70, 30, 120, 5}}});
    c.push_back({ScenarioId::Obstacle, "obstacle", "Obstacle ahead",
                 "The driver keeps the throttle pressed towards a stationary obstacle; AEB must stop the car.",
                 {{"speed", "Start speed", "km/h", 80, 20, 130, 5},
                  {"distance", "Obstacle distance", "m", 150, 40, 300, 10},
                  {"driverThrottle", "Driver throttle", "", 1.0, 0.0, 1.0, 0.1},
                  {"aebArmed", "AEB armed (1 = on)", "", 1, 0, 1, 1},
                  {"obstacleLane", "Obstacle lane (0 left, 1 ego, 2 right)", "", 1, 0, 2, 1}}});
    c.push_back({ScenarioId::Pedestrian, "pedestrian", "Pedestrian crossing",
                 "A pedestrian steps out from the right kerb and walks across the road in front of the car.",
                 {{"speed", "Start speed", "km/h", 50, 20, 90, 5},
                  {"distance", "Crossing distance", "m", 100, 40, 200, 10},
                  {"pedSpeed", "Pedestrian speed", "m/s", 1.4, 0.5, 3.0, 0.1},
                  {"triggerTtc", "Steps out at TTC", "s", 4.5, 2.0, 6.0, 0.5},
                  {"autonomous", "Autonomous mode (1 = on)", "", 1, 0, 1, 1},
                  {"driverThrottle", "Driver throttle (manual)", "", 0.2, 0.0, 1.0, 0.1},
                  {"aebArmed", "AEB armed (1 = on)", "", 1, 0, 1, 1}}});
    c.push_back({ScenarioId::Parking, "parking", "Automated parking",
                 "The car finds a free bay in a lot on the right and reverses into it.",
                 {{"startX", "Start lateral position", "m", 3.6, -3.6, 6.0, 0.2},
                  {"startSpeed", "Start speed", "km/h", 10, 0, 60, 5},
                  {"bayDistance", "Distance to first free bay", "m", 30, 10, 38, 2},
                  {"freeBays", "Free bays (1 both, 2 bay 4, 3 bay 7)", "", 1, 1, 3, 1},
                  {"maxOffset", "Allowed final offset", "m", 0.4, 0.2, 1.0, 0.1},
                  {"timeout", "Time limit", "s", 120, 30, 240, 10}}});
    c.push_back({ScenarioId::Overtake, "overtake", "Autonomous overtaking",
                 "Autonomous mode catches a slower car, overtakes it in a free lane, and returns to its own lane.",
                 {{"accSpeed", "Set speed (also start speed)", "km/h", 100, 60, 130, 5},
                  {"leadSpeed", "Slower car's speed", "km/h", 60, 20, 90, 5},
                  {"gap", "Initial gap", "m", 100, 60, 200, 10},
                  {"blockLeft", "A car alongside on the left (1 = yes)", "", 0, 0, 1, 1},
                  {"duration", "Time limit", "s", 90, 40, 150, 5}}});
    c.push_back({ScenarioId::LaneAvoid, "laneavoid", "Obstacle avoidance by lane change",
                 "A stationary obstacle appears ahead in the lane. Autonomous mode changes lane around it, or stops if no lane is free.",
                 {{"speed", "Speed", "km/h", 80, 50, 120, 5},
                  {"distance", "Obstacle appears this far ahead", "m", 110, 60, 140, 5},
                  {"appearAt", "Appears at", "s", 4, 1, 20, 1},
                  {"blockSides", "Cars alongside in both side lanes (1 = yes)", "", 0, 0, 1, 1},
                  {"duration", "Time limit", "s", 60, 30, 90, 5}}});
    return c;
}
}  // namespace

const std::vector<ScenarioInfo>& scenarioCatalog() {
    static const std::vector<ScenarioInfo> catalog = buildCatalog();
    return catalog;
}

const ScenarioInfo& scenarioInfo(ScenarioId id) { return scenarioCatalog().at(static_cast<std::size_t>(id)); }

bool scenarioIdFromKey(const std::string& key, ScenarioId* id) {
    for (const ScenarioInfo& s : scenarioCatalog()) {
        if (s.key == key) {
            if (id) *id = s.id;
            return true;
        }
    }
    return false;
}

ParamMap defaultParams(ScenarioId id) {
    ParamMap m;
    for (const ParamSpec& p : scenarioInfo(id).params) m[p.key] = p.def;
    return m;
}

ParamMap resolveParams(ScenarioId id, const ParamMap& overrides) {
    ParamMap m;
    for (const ParamSpec& p : scenarioInfo(id).params) {
        double v = p.def;
        const auto it = overrides.find(p.key);
        if (it != overrides.end()) v = it->second;
        m[p.key] = std::min(std::max(v, p.min), p.max);
    }
    return m;
}

std::string ScenarioReport::outcomeText() const {
    switch (outcome) {
    case Outcome::Pass: return "PASS";
    case Outcome::Fail: return "FAIL";
    case Outcome::Aborted: return "ABORTED";
    }
    return "?";
}

std::string ScenarioReport::toMarkdown() const {
    const ScenarioInfo& info = scenarioInfo(id);
    int ok = 0;
    for (const Criterion& c : criteria) ok += c.passed ? 1 : 0;

    std::string md = "# Scenario report: " + name + "\n\n";
    md += "**Result: " + outcomeText() + "**";
    if (outcome != Outcome::Aborted) md += " (" + std::to_string(ok) + " of " + std::to_string(criteria.size()) + " criteria met)";
    md += "\n\n";
    md += "- Simulated duration: " + num(duration, 1) + " s\n";
    md += "- Ended because: " + endReason + "\n";
    md += "- Data: **SIMULATED** (no real-world measurements)\n\n";

    md += "## Parameters\n\n| Parameter | Value |\n|---|---|\n";
    for (const ParamSpec& p : info.params) {
        const auto it = params.find(p.key);
        if (it == params.end()) continue;
        md += "| " + p.label + " | " + num(it->second, p.step < 1.0 ? 1 : 0) + (p.unit.empty() ? "" : " " + p.unit) + " |\n";
    }

    md += "\n## Criteria\n\n| Criterion | Requirement | Measured | Result |\n|---|---|---|---|\n";
    for (const Criterion& c : criteria)
        md += "| " + c.name + " | " + c.requirement + " | " + c.measured + " | " + (c.passed ? "PASS" : "FAIL") + " |\n";

    if (!notes.empty()) {
        md += "\n## Notes\n\n";
        for (const std::string& n : notes) md += "- " + n + "\n";
    }
    md += "\n## Metrics\n\n| Metric | Value |\n|---|---|\n";
    for (const auto& kv : metrics) md += "| " + kv.first + " | " + num(kv.second, 3) + " |\n";
    md += "\n> Educational simulator, not a safety-certified automotive system.\n";
    return md;
}

std::string ScenarioReport::toJson() const {
    std::string j = "{\n";
    j += "  \"scenario\": " + jsonString(scenarioInfo(id).key) + ",\n";
    j += "  \"name\": " + jsonString(name) + ",\n";
    j += "  \"result\": " + jsonString(outcomeText()) + ",\n";
    j += "  \"duration_s\": " + num(duration, 3) + ",\n";
    j += "  \"ended_because\": " + jsonString(endReason) + ",\n";
    j += "  \"data_source\": \"simulated\",\n";
    j += "  \"parameters\": {";
    bool first = true;
    for (const auto& kv : params) {
        j += std::string(first ? "" : ",") + "\n    " + jsonString(kv.first) + ": " + num(kv.second, 3);
        first = false;
    }
    j += "\n  },\n  \"criteria\": [";
    first = true;
    for (const Criterion& c : criteria) {
        j += std::string(first ? "" : ",") + "\n    {\"name\": " + jsonString(c.name) + ", \"requirement\": " + jsonString(c.requirement) +
             ", \"measured\": " + jsonString(c.measured) + ", \"passed\": " + (c.passed ? "true" : "false") + "}";
        first = false;
    }
    j += "\n  ],\n  \"metrics\": {";
    first = true;
    for (const auto& kv : metrics) {
        j += std::string(first ? "" : ",") + "\n    " + jsonString(kv.first) + ": " + num(kv.second, 6);
        first = false;
    }
    j += "\n  }\n}\n";
    return j;
}

bool ScenarioReport::sameResultAs(const ScenarioReport& o) const {
    return id == o.id && outcome == o.outcome && duration == o.duration && metrics == o.metrics;
}

}  // namespace driveforge
