#include "icons.h"

QIcon Icons::getIcon(Icons::IconId id) {
  auto icons = getIcons();
  if (icons.contains(id)) {
    return icons[id];
  } else {
    return QIcon();
  }
}


QMap<Icons::IconId, QIcon>& Icons::getIcons() {
  static QMap<Icons::IconId, QIcon> icons;
  if (icons.empty()) {
    icons.insert(Icons::kHexCrystalNormal, QIcon(":/icons/hex_crystal_normal_64.png"));
    icons.insert(Icons::kHexCrystalOn, QIcon(":/icons/hex_crystal_on_64.png"));
    icons.insert(Icons::kHexCloseNormal, QIcon(":/icons/hex_close_normal_64.png"));
    icons.insert(Icons::kHexCloseOn, QIcon(":/icons/hex_close_on_64.png"));
    icons.insert(Icons::kCloseNormal, QIcon(":/icons/icon_close_checked_normal_32.png"));
    icons.insert(Icons::kCloseOn, QIcon(":/icons/icon_close_on_32.png"));
    icons.insert(Icons::kHexAddOn, QIcon(":/icons/hex_add_on.png"));
    icons.insert(Icons::kLink, QIcon(":/icons/link.png"));
    icons.insert(Icons::kUnlink, QIcon(":/icons/unlink.png"));
  }
  return icons;
}
