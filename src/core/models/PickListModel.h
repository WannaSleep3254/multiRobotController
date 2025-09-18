#ifndef PICKLISTMODEL_H
#define PICKLISTMODEL_H

#include <QAbstractTableModel>

struct PickPose {
    double x,y,z, rx,ry,rz;
};

class PickListModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    explicit PickListModel(QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& idx, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

    void add(const PickPose& p);
    void clear();

    PickPose getRow(int r) const;

private:
    QVector<PickPose> m_data;
};

#endif // PICKLISTMODEL_H
