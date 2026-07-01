/******************************************************************************
 * Copyright 2025 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
enum DEBUG_LEVEL_NFC {
	DEBUG_OFF_NFC,
	DEBUG_ON_NFC
};

#define print_debug(msg...) \
switch (IS_ENABLED(CONFIG_DYNAMIC_DEBUG)) \
{ \
	case DEBUG_OFF_NFC: \
		break; \
	case DEBUG_ON_NFC: \
		pr_debug(msg); \
		break; \
} \

#define device_debug(ptr, msg...) \
switch (IS_ENABLED(CONFIG_DYNAMIC_DEBUG)) \
{ \
	case DEBUG_OFF_NFC: \
		break; \
	case DEBUG_ON_NFC: \
		dev_dbg(ptr, msg); \
		break; \
}
