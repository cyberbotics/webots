// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_MULTIPLE_VALUE_HPP
#define WB_MULTIPLE_VALUE_HPP

//
// Description: abstract base class for all multiple values
//
// Inherited by:
//   WbMFVector3, WbMFVector2, WbMFString, WbMFNode, WbMFInt, WbMFDouble, WbMFBool, WbMFRotation, WbMFColor
//

#include "WbValue.hpp"
#include "WbVariant.hpp"

class WbMultipleValue : public WbValue {
  Q_OBJECT

public:
  virtual ~WbMultipleValue() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override;
  virtual int size() const = 0;
  bool isEmpty() const { return size() == 0; };
  virtual void clear() = 0;
  virtual void removeItem(int index) = 0;
  virtual void insertDefaultItem(int index) = 0;
  virtual WbVariant defaultNewVariant() const = 0;
  bool valueAtIndexEqualsSingleValue(int index, const WbValue *other) const;
  QString toString(WbPrecision::Level level) const override;
  // level is not meaningful in all the subclasses.
  QString itemToString(int index, WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const;
  void write(WbWriter &writer) const override;
  virtual void writeItem(WbWriter &writer, int index) const = 0;

  // return generic value
  virtual WbVariant variantValue(int index) const = 0;

signals:
  void itemChanged(int index);   // called when a single item is modified
  void itemRemoved(int index);   // called when a single item is removed
  void itemInserted(int index);  // called when a single item is inserted
  void cleared();                // called when all items are removed

protected:
  WbMultipleValue() {}
  virtual void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) = 0;
  virtual bool smallSeparator(int i) const { return size() > 25; }

private:
};

// template iterator class use by all WbMF containers
template<class MF, class T> class WbMFIterator {
public:
  explicit WbMFIterator(const MF &mf) : i(-1), mMf(&mf) {}
  explicit WbMFIterator(const MF *mf) : i(-1), mMf(mf) {}
  WbMFIterator(const WbMFIterator &other) : i(other.i), mMf(other.mMf) {}
  WbMFIterator &operator=(const WbMFIterator &other) {
    i = other.i;
    mMf = other.mMf;
    return *this;
  }
  bool hasNext() const { return i + 1 < mMf->size(); }
  bool hasPrevious() const { return i > 0; }

  const T &peekNext() const { return mMf->item(i + 1); }
  const T &next() { return mMf->item(++i); }
  const T &peekPrevious() const { return mMf->item(i - 1); }
  const T &previous() { return mMf->item(--i); }
  void toBack() { i = mMf->size(); }
  void toFront() { i = -1; }

private:
  int i;
  const MF *mMf;

  WbMFIterator() {
    i = 0;
    mMf = NULL;
  }
};

#endif
