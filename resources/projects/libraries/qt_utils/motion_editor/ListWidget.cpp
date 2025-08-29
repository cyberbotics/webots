#include "ListWidget.hpp"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

#include <cassert>

using namespace webotsQtUtils;

ListWidget::ListWidget(bool showAddRemove, bool showUpDown, bool showReset, bool showDuplicate, QWidget *parent) :
  QGroupBox(parent) {
  setObjectName("borderedGroupBox");
  setTitle(tr("List widget"));

  mMainLayout = new QGridLayout(this);
  QWidget *mainWidget = new QWidget(this);

  QWidget *buttonsWidget = new QWidget(this);
  QVBoxLayout *vBoxLayout = new QVBoxLayout(mainWidget);
  QHBoxLayout *hBoxLayout = new QHBoxLayout(buttonsWidget);

  mListWidget = new QListWidget(this);
  connect(mListWidget, &QListWidget::itemSelectionChanged, this, &ListWidget::updateButtons);

  mUpButton = new QPushButton(this);
  mUpButton->setVisible(showUpDown);
  mUpButton->setIcon(QIcon("icons:up_button.png"));
  mUpButton->setToolTip(tr("Move the selected item(s) up"));
  connect(mUpButton, &QPushButton::pressed, this, &ListWidget::up);

  mDownButton = new QPushButton(this);
  mDownButton->setVisible(showUpDown);
  mDownButton->setIcon(QIcon("icons:down_button.png"));
  mDownButton->setToolTip(tr("Move the selected item(s) down"));
  connect(mDownButton, &QPushButton::pressed, this, &ListWidget::down);

  mResetButton = new QPushButton(this);
  mResetButton->setVisible(showReset);
  mResetButton->setIcon(QIcon("icons:reset_button.png"));
  mResetButton->setToolTip(tr("Reset value to saved one"));

  mDuplicateButton = new QPushButton(this);
  mDuplicateButton->setVisible(showDuplicate);
  mDuplicateButton->setIcon(QIcon("icons:duplicate_button.png"));
  mDuplicateButton->setToolTip(tr("Duplicate selected item"));
  mDuplicateButton->setShortcut(Qt::CTRL | Qt::Key_D);
  connect(mDuplicateButton, &QPushButton::pressed, this, &ListWidget::duplicate);

  mAddButton = new QPushButton(this);
  mAddButton->setVisible(showAddRemove);
  mAddButton->setIcon(QIcon("icons:plus.png"));
  mAddButton->setToolTip(tr("Add a new item"));
  connect(mAddButton, &QPushButton::pressed, this, &ListWidget::add);

  mRemoveButton = new QPushButton(this);
  mRemoveButton->setVisible(showAddRemove);
  mRemoveButton->setIcon(QIcon("icons:minus.png"));
  mRemoveButton->setToolTip(tr("Remove the selected item(s)"));
  connect(mRemoveButton, &QPushButton::pressed, this, &ListWidget::remove);

  hBoxLayout->addWidget(mUpButton);
  hBoxLayout->addWidget(mDownButton);
  hBoxLayout->addWidget(mResetButton);
  hBoxLayout->addWidget(mDuplicateButton);
  hBoxLayout->addWidget(mAddButton);
  hBoxLayout->addWidget(mRemoveButton);

  vBoxLayout->addWidget(mListWidget);
  vBoxLayout->addWidget(buttonsWidget);

  // code to put the buttons as top-right as possible, without spacing
  mUpButton->setMaximumSize(24, 24);
  mDownButton->setMaximumSize(24, 24);
  mDuplicateButton->setMaximumSize(24, 24);
  mAddButton->setMaximumSize(24, 24);
  mRemoveButton->setMaximumSize(24, 24);
  mResetButton->setMaximumSize(24, 24);
  hBoxLayout->setAlignment(Qt::AlignRight | Qt::AlignTop);
#ifdef __APPLE__
  hBoxLayout->setSpacing(12);  // button size / 2
#else
  hBoxLayout->setSpacing(0);
#endif
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  vBoxLayout->setAlignment(Qt::AlignHCenter | Qt::AlignBottom);
  vBoxLayout->setSpacing(0);

  mMainLayout->addWidget(mainWidget, 0, 0);

  // set a monospace font - This part could certainly be improved
#ifdef _WIN32
  QFont f("Consolas,10");
#elif defined(__APPLE__)
  QFont f("Courier,14");
#else
  QFont f("Monospace");
#endif
  mListWidget->setFont(f);

  updateButtons();
}

ListWidget::~ListWidget() {
}

void ListWidget::updateButtons() {
  const QList<QListWidgetItem *> &selectedItems = mListWidget->selectedItems();
  if (selectedItems.size() == 0) {
    mUpButton->setEnabled(false);
    mDownButton->setEnabled(false);
    mDuplicateButton->setEnabled(false);
    mAddButton->setEnabled(true);
    mRemoveButton->setEnabled(false);
    mResetButton->setEnabled(false);
  } else {  // selectedItems.size() == 1
    int row = mListWidget->currentRow();
    mUpButton->setEnabled(row != 0);
    mDownButton->setEnabled(row != mListWidget->count() - 1);
    mDuplicateButton->setEnabled(true);
    mAddButton->setEnabled(true);
    mRemoveButton->setEnabled(true);
  }
}

void ListWidget::up() {
  assert(mListWidget->selectedItems().size() == 1);

  int row = mListWidget->currentRow();
  if (row > 0 && row < mListWidget->count()) {
    swapItemWithNext(row - 1);
    updateButtons();
  }
}

void ListWidget::down() {
  assert(mListWidget->selectedItems().size() == 1);

  int row = mListWidget->currentRow();
  if (row >= 0 && row < mListWidget->count() - 1) {
    swapItemWithNext(row);
    mListWidget->setCurrentRow(row + 1);
    updateButtons();
  }
}

void ListWidget::add() {
  bool firstItem = count() == 0;

  if (mListWidget->selectedItems().size() == 1)
    newItemAt(mListWidget->currentRow() + 1);
  else
    newItemAt(0);

  if (firstItem)
    mListWidget->setCurrentRow(0);

  updateButtons();
}

void ListWidget::remove() {
  assert(mListWidget->selectedItems().size() == 1);

  int row = mListWidget->currentRow();
  deleteItemAt(row);

  updateButtons();
}

void ListWidget::duplicate() {
  assert(mListWidget->selectedItems().size() == 1);

  duplicateItemAt(mListWidget->currentRow());

  updateButtons();
}

int ListWidget::currentIndex() const {
  return mListWidget->currentRow();
}
