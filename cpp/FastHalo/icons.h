#ifndef ICONS_H
#define ICONS_H

#include <QIcon>
#include <QMap>

class Icons {
 public:
  Icons() = delete;

  enum IconId {
    kHexCloseNormal,
    kHexCloseOn,
    kHexCrystalNormal,
    kHexCrystalOn,
    kHexAddOn,
    kCloseNormal,
    kCloseOn,
    kLink,
    kUnlink,
  };

  static QIcon getIcon(IconId id);

 private:
  static QMap<IconId, QIcon>& getIcons();
};

#endif  // ICONS_H
