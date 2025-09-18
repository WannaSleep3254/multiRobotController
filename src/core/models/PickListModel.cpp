#include "PickListModel.h"
#include <QVariant>

PickListModel::PickListModel(QObject *parent)
    : QAbstractTableModel(parent)
    , m_data({
        {100.0,  50.0, 300.0, 180.0, 0.0, 90.0},
        {120.0,  55.0, 302.0, 180.0, 0.0, 90.0},
        {140.0,  60.0, 305.0, 180.0, 0.0, 90.0}
    })
{

}

int PickListModel::rowCount(const QModelIndex& parent) const {
    Q_UNUSED(parent);
    return m_data.size();
}

int PickListModel::columnCount(const QModelIndex& parent) const {
    Q_UNUSED(parent);
    return 6;
}

QVariant PickListModel::data(const QModelIndex& idx, int role) const {
    if (!idx.isValid() || idx.row() < 0 || idx.row() >= m_data.size())
        return {};

    if (role == Qt::DisplayRole) {
        const auto& p = m_data[idx.row()];
        switch (idx.column()) {
        case 0: return p.x;
        case 1: return p.y;
        case 2: return p.z;
        case 3: return p.rx;
        case 4: return p.ry;
        case 5: return p.rz;
        default: break;
        }
    }
    return {};
}

QVariant PickListModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole)
        return {};

    if (orientation == Qt::Horizontal) {
        static const char* H[] = {"X(mm)", "Y(mm)", "Z(mm)", "Rx(deg)", "Ry(deg)", "Rz(deg)"};
        if (section >= 0 && section < 6)
            return H[section];
        return {};
    }
    return section + 1;
}

void PickListModel::add(const PickPose& p) {
    const int row = m_data.size();
    beginInsertRows(QModelIndex(), row, row);
    m_data.push_back(p);
    endInsertRows();
}

void PickListModel::clear() {
    beginResetModel();
    m_data.clear();
    endResetModel();
}

PickPose PickListModel::getRow(int r) const {
    // 필요시 범위 체크 강화
    return m_data[r];
}
