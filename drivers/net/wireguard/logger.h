#ifndef _WG_LOGGER_H
#define _WG_LOGGER_H

void wg_log(int level, const struct net_device *dev, const struct wg_peer *peer, const char *fmt, ...);

int wg_logger_init(void);
void wg_logger_uninit(void);

#endif /* _WG_LOGGER_H */
