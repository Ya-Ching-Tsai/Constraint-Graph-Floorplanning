#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <deque>
#include <string>
#include <unordered_map>
#include <algorithm>

using namespace std;

// 定義模組 Block 結構，包含名稱、寬度與高度
struct Block {
    string id;
    int width, height;
};

// Floorplanner 類別封裝整個流程
class Floorplanner {
public:
    // 建構時自動執行整個流程
    Floorplanner(const string& filename) : baseName(filename) {
        readInput();               // 讀取輸入檔案，建立圖與模組資訊
        computeEarliestPositions(); // 正向拓撲排序求 earliest X/Y
        computeLatestPositions();   // 反向拓撲排序求 latest X/Y
        findCriticalEdges();        // 找出關鍵邊（關鍵路徑上的模組連接）
        tryAreaReduction();         // 嘗試移除水平關鍵邊→變為垂直邊，減少面積
        writeOutput();              // 輸出結果
    }

private:
    string baseName;                       // 檔名（不含 .txt）
    vector<Block> blocks;                  // 模組列表
    unordered_map<string, int> idToIdx;    // 模組名稱對應索引
    vector<vector<int>> hGraph, vGraph;    // 水平/垂直圖（HCG/VCG）
    vector<int> startX, startY, endX, endY; // 每個模組 earliest/latest X/Y
    vector<pair<int, int>> hCritEdges, vCritEdges; // 關鍵邊
    long long minArea = 0;                 // 最小面積

    // 各階段函式
    void readInput();
    void computeEarliestPositions();
    void computeLatestPositions();
    void findCriticalEdges();
    void tryAreaReduction();
    void writeOutput();

    // 拓撲排序通用工具（水平或垂直）
    void topoForward(const vector<vector<int>>&, const vector<Block>&, vector<int>&, bool);
    void topoBackward(const vector<vector<int>>&, const vector<Block>&, vector<int>&, int, bool);
};

// 讀取輸入檔案，並建構模組資訊與邊圖
void Floorplanner::readInput() {
    ifstream fin(baseName + ".txt");
    if (!fin.is_open()) {
        cerr << "Cannot open file: " << baseName << ".txt\n";
        exit(1);
    }

    string line;
    int moduleCount = 0;

    // 解析模組數量
    while (getline(fin, line)) {
        if (line.find("number of modules") != string::npos) {
            moduleCount = stoi(line.substr(line.find(":") + 1));
            break;
        }
    }

    blocks.resize(moduleCount);

    // 找到模組尺寸段落
    while (getline(fin, line)) {
        if (line.find("module dimension") != string::npos) break;
    }

    // 解析每個模組的資料
    for (int i = 0; i < moduleCount; ++i) {
        getline(fin, line);
        string name;
        stringstream ss(line);
        string ignore;
        ss >> ignore >> name;
        auto a = line.find('('), b = line.find(','), c = line.find(')');
        blocks[i] = {
            name,
            stoi(line.substr(a + 1, b - a - 1)),
            stoi(line.substr(b + 1, c - b - 1))
        };
        idToIdx[name] = i;
    }

    // 初始化圖
    hGraph.assign(moduleCount, {});
    vGraph.assign(moduleCount, {});

    // 解析 HCG 邊
    while (getline(fin, line)) if (line.find("edges in HCG") != string::npos) break;
    getline(fin, line);
    stringstream sh(line);
    string token;
    while (getline(sh, token, ',')) {
        auto mid = token.find("to");
        string u = token.substr(0, mid), v = token.substr(mid + 2);
        u.erase(remove_if(u.begin(), u.end(), ::isspace), u.end());
        v.erase(remove_if(v.begin(), v.end(), ::isspace), v.end());
        hGraph[idToIdx[u]].push_back(idToIdx[v]);
    }

    // 解析 VCG 邊
    while (getline(fin, line)) if (line.find("edges in VCG") != string::npos) break;
    getline(fin, line);
    stringstream sv(line);
    while (getline(sv, token, ',')) {
        auto mid = token.find("to");
        string u = token.substr(0, mid), v = token.substr(mid + 2);
        u.erase(remove_if(u.begin(), u.end(), ::isspace), u.end());
        v.erase(remove_if(v.begin(), v.end(), ::isspace), v.end());
        vGraph[idToIdx[u]].push_back(idToIdx[v]);
    }
}

// 正向拓撲排序：從前驅節點推算 earliest 擺放位置
void Floorplanner::topoForward(const vector<vector<int>>& graph, const vector<Block>& blocks,
                               vector<int>& coords, bool horizontal) {
    int n = graph.size();
    coords.assign(n, 0);
    vector<int> indegree(n);
    for (int u = 0; u < n; ++u)
        for (int v : graph[u]) indegree[v]++;

    deque<int> q;
    for (int i = 0; i < n; ++i)
        if (indegree[i] == 0) q.push_back(i);

    while (!q.empty()) {
        int u = q.front(); q.pop_front();
        for (int v : graph[u]) {
            int shift = horizontal ? blocks[u].width : blocks[u].height;
            coords[v] = max(coords[v], coords[u] + shift); // earliest propagate
            if (--indegree[v] == 0) q.push_back(v);
        }
    }
}

// 反向拓撲排序：從終點模組往前推算 latest 可擺放位置
void Floorplanner::topoBackward(const vector<vector<int>>& graph, const vector<Block>& blocks,
                                vector<int>& latest, int bound, bool horizontal) {
    int n = graph.size();
    latest.assign(n, bound);
    vector<int> outdeg(n);
    vector<vector<int>> rev(n);
    for (int u = 0; u < n; ++u)
        for (int v : graph[u]) {
            rev[v].push_back(u);
            outdeg[u]++;
        }

    deque<int> q;
    for (int i = 0; i < n; ++i) {
        latest[i] = bound - (horizontal ? blocks[i].width : blocks[i].height);
        if (outdeg[i] == 0) q.push_back(i);
    }

    while (!q.empty()) {
        int u = q.front(); q.pop_front();
        for (int p : rev[u]) {
            int shift = horizontal ? blocks[p].width : blocks[p].height;
            latest[p] = min(latest[p], latest[u] - shift);
            if (--outdeg[p] == 0) q.push_back(p);
        }
    }
}

// 正向拓撲排序求 earliest X/Y
void Floorplanner::computeEarliestPositions() {
    topoForward(hGraph, blocks, startX, true);   // X軸
    topoForward(vGraph, blocks, startY, false);  // Y軸
}

// 反向拓撲排序求 latest X/Y
void Floorplanner::computeLatestPositions() {
    int maxX = 0, maxY = 0;
    for (int i = 0; i < blocks.size(); ++i) {
        maxX = max(maxX, startX[i] + blocks[i].width);
        maxY = max(maxY, startY[i] + blocks[i].height);
    }
    minArea = 1LL * maxX * maxY; // 初始面積
    topoBackward(hGraph, blocks, endX, maxX, true);
    topoBackward(vGraph, blocks, endY, maxY, false);
}

// 找出水/垂直方向的關鍵邊（模組在 earliest = latest 上）
void Floorplanner::findCriticalEdges() {
    int n = blocks.size();
    for (int u = 0; u < n; ++u) {
        for (int v : hGraph[u])
            if (startX[u] == endX[u] && startX[v] == endX[v])
                hCritEdges.emplace_back(u, v);
        for (int v : vGraph[u])
            if (startY[u] == endY[u] && startY[v] == endY[v])
                vCritEdges.emplace_back(u, v);
    }
}

// 嘗試把每個水平關鍵邊換成垂直邊，若能縮小面積就更新 minArea
void Floorplanner::tryAreaReduction() {
    int n = blocks.size();
    for (const auto& p : hCritEdges) {
        int u = p.first;
        int v = p.second;

        auto hG = hGraph, vG = vGraph;
        hG[u].erase(remove(hG[u].begin(), hG[u].end(), v), hG[u].end());
        if (find(vG[u].begin(), vG[u].end(), v) == vG[u].end())
            vG[u].push_back(v);

        vector<int> x, y;
        topoForward(hG, blocks, x, true);
        topoForward(vG, blocks, y, false);
        int newW = 0, newH = 0;
        for (int i = 0; i < n; ++i) {
            newW = max(newW, x[i] + blocks[i].width);
            newH = max(newH, y[i] + blocks[i].height);
        }
        minArea = min(minArea, 1LL * newW * newH);
    }
}


// 輸出結果到指定格式的 .txt 檔
void Floorplanner::writeOutput() {
    ofstream fout(baseName + "_N26134235.txt");
    fout << "number of horizontal critical edges " << hCritEdges.size() << "\n";
    for (const auto& p : hCritEdges) {
        int u = p.first;
        int v = p.second;
        fout << blocks[u].id << " to " << blocks[v].id << "\n";
    }

    fout << "\nnumber of vertical critical edges " << vCritEdges.size() << "\n";
    for (const auto& p : vCritEdges) {
        int u = p.first;
        int v = p.second;
        fout << blocks[u].id << " to " << blocks[v].id << "\n";
    }

    fout << "\nminimum floorplan area " << minArea << "";
}


// 主程式：讀檔名並執行 Floorplanner
int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: ./floorplanner <input_file_without_extension>\n";
        return 1;
    }
    Floorplanner planner(argv[1]);
    return 0;
}
