#ifndef PICKLISTMODEL_H
#define PICKLISTMODEL_H

#include <QAbstractTableModel>
#include "Pose6D.h"

class PickListModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    explicit PickListModel(QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& idx, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

    void add(const Pose6D& p);
    void clear();

    Pose6D getRow(int r) const;

    void setActiveRow(int r); // 선택 행 강조 표시)

private:
    int m_activeRow{-1};

private:
    QVector<Pose6D> m_data;
};

#endif // PICKLISTMODEL_H
