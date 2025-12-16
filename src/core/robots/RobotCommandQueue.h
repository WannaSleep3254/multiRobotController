#ifndef ROBOTCOMMANDQUEUE_H
#define ROBOTCOMMANDQUEUE_H

#include <functional>

#include <QObject>
#include <QQueue>
#include <QList>

class RobotCommandQueue : public QObject {
    Q_OBJECT
public:
    using Step = std::function<void(std::function<void(bool)> done)>;

    explicit RobotCommandQueue(QObject* parent=nullptr) : QObject(parent) {}

    void enqueue(const QList<Step>& steps) {
        m_q.enqueue(steps);
        if (!m_running) runNext();
    }

private:
    void runNext() {
        if (m_q.isEmpty()) { m_running=false; return; }
        m_running=true;
        m_steps = m_q.dequeue();
        m_idx = 0;
        runStep();
    }
    void runStep() {
        if (m_idx >= m_steps.size()) { runNext(); return; }
        auto step = m_steps[m_idx++];
        step([this](bool ok){
            if (!ok) { m_q.clear(); m_running=false; return; }
            runStep();
        });
    }

    QQueue<QList<Step>> m_q;
    QList<Step> m_steps;
    int m_idx = 0;
    bool m_running = false;
};

#endif // ROBOTCOMMANDQUEUE_H
