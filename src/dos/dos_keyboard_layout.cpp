/*
 *  Copyright (C) 2002-2021  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include "dosbox.h"

#include <string_view>
using sv = std::string_view;

#include "bios.h"
#include "bios_disk.h"
#include "setup.h"
#include "support.h"
#include "string_utils.h"
#include "../ints/int10.h"
#include "regs.h"
#include "callback.h"
#include "mapper.h"
#include "drives.h"
#include "dos_inc.h"

#include "dos_keyboard_layout.h"
#include "dos_resources.h"

#if defined (WIN32)
#include <windows.h>
#endif
#include <map>

static FILE_unique_ptr OpenDosboxFile(const char *name)
{
	uint8_t drive;
	char fullname[DOS_PATHLENGTH];

	localDrive* ldp=0;
	// try to build dos name
	if (DOS_MakeName(name,fullname,&drive)) {
		try {
			// try to open file on mounted drive first
			ldp=dynamic_cast<localDrive*>(Drives[drive]);
			if (ldp) {
				FILE *tmpfile=ldp->GetSystemFilePtr(fullname, "rb");
				if (tmpfile != NULL)
					return FILE_unique_ptr(tmpfile);
			}
		}
		catch(...) {}
	}
	FILE *tmpfile=fopen(name, "rb");
	return FILE_unique_ptr(tmpfile);
}

class keyboard_layout {
public:
	keyboard_layout()
	        : additional_planes(0),
	          used_lock_modifiers(0x0f),
	          diacritics_entries(0),
	          diacritics_character(0),
	          user_keys(0),
	          use_foreign_layout(false),
	          language_codes(nullptr),
	          language_code_count(0)
	{
		this->reset();
		sprintf(current_keyboard_file_name, "none");
	}

	keyboard_layout(const keyboard_layout &) = delete; // prevent copying
	keyboard_layout &operator=(const keyboard_layout &) = delete; // prevent assignment

	~keyboard_layout();

	// read in a codepage from a .cpi-file
	Bitu read_codepage_file(const char* codepage_file_name, int32_t codepage_id);
	uint16_t extract_codepage(const char* keyboard_file_name);
	// read in a keyboard layout from a .kl-file
	Bitu read_keyboard_file(const char* keyboard_file_name, int32_t req_cp);

	// call layout_key to apply the current language layout
	bool layout_key(Bitu key, uint8_t flags1, uint8_t flags2, uint8_t flags3);

	Bitu switch_keyboard_layout(const char* new_layout, keyboard_layout* &created_layout, int32_t& tried_cp);
	void switch_foreign_layout();
	const char* get_layout_name();
	const char* main_language_code();


private:
	static const uint8_t layout_pages=12;
	uint16_t current_layout[(MAX_SCAN_CODE+1)*layout_pages];
	struct {
		uint16_t required_flags,forbidden_flags;
		uint16_t required_userflags,forbidden_userflags;
	} current_layout_planes[layout_pages-4];
	uint8_t additional_planes,used_lock_modifiers;

	// diacritics table
	uint8_t diacritics[2048];
	uint16_t diacritics_entries;
	uint16_t diacritics_character;
	uint16_t user_keys;

	char current_keyboard_file_name[256];
	bool use_foreign_layout;

	// language code storage used when switching layouts
	char** language_codes;
	Bitu language_code_count;

	void reset();
	void read_keyboard_file(int32_t specific_layout);
	Bitu read_keyboard_file(const char* keyboard_file_name, int32_t specific_layout, int32_t requested_codepage);
	bool map_key(Bitu key, uint16_t layouted_key, bool is_command, bool is_keypair);
	int8_t get_CPX_file_id(uint32_t codepage_id);
};

keyboard_layout::~keyboard_layout() {
	if (language_codes) {
		for (Bitu i=0; i<language_code_count; i++)
			delete[] language_codes[i];
		delete[] language_codes;
		language_codes=NULL;
	}
}

void keyboard_layout::reset() {
	for (uint32_t i=0; i<(MAX_SCAN_CODE+1)*layout_pages; i++) current_layout[i]=0;
	for (uint32_t i=0; i<layout_pages-4; i++) {
		current_layout_planes[i].required_flags=0;
		current_layout_planes[i].forbidden_flags=0xffff;
		current_layout_planes[i].required_userflags=0;
		current_layout_planes[i].forbidden_userflags=0xffff;
	}
	used_lock_modifiers=0x0f;
	diacritics_entries=0;		// no diacritics loaded
	diacritics_character=0;
	user_keys=0;				// all userkeys off
	language_code_count=0;
}

Bitu keyboard_layout::read_keyboard_file(const char* keyboard_file_name, int32_t req_cp) {
	return this->read_keyboard_file(keyboard_file_name, -1, req_cp);
}

// switch to a different layout
void keyboard_layout::read_keyboard_file(int32_t specific_layout) {
	if (strcmp(current_keyboard_file_name,"none"))
		this->read_keyboard_file(current_keyboard_file_name, specific_layout, dos.loaded_codepage);
}

static uint32_t read_kcl_file(const char* kcl_file_name, const char* layout_id, bool first_id_only) {
	auto tempfile = OpenDosboxFile(kcl_file_name);
	if (!tempfile)
		return 0;

	static uint8_t rbuf[8192];

	// check ID-bytes of file
	uint32_t dr = (uint32_t)fread(rbuf, sizeof(uint8_t), 7, tempfile.get());
	if ((dr < 7) || (rbuf[0] != 0x4b) || (rbuf[1] != 0x43) || (rbuf[2] != 0x46)) {
		return 0;
	}

	const auto seek_pos = 7 + rbuf[6];
	if (fseek(tempfile.get(), seek_pos, SEEK_SET) != 0) {
		LOG_WARNING("LAYOUT: could not seek to byte %d in keyboard layout file '%s': %s",
		            seek_pos, kcl_file_name, strerror(errno));
		return 0;
	}

	for (;;) {
		uint32_t cur_pos = (uint32_t)(ftell(tempfile.get()));
		dr = (uint32_t)fread(rbuf, sizeof(uint8_t), 5, tempfile.get());
		if (dr < 5)
			break;
		uint16_t len=host_readw(&rbuf[0]);

		uint8_t data_len=rbuf[2];
		assert (data_len < UINT8_MAX);
		static_assert(UINT8_MAX < sizeof(rbuf), "rbuf too small");

		char lng_codes[258];
		fseek(tempfile.get(), -2, SEEK_CUR);
		// get all language codes for this layout
		for (Bitu i=0; i<data_len;) {
			if (fread(rbuf, sizeof(uint8_t), 2, tempfile.get()) != 2) {
				break;
			}
			uint16_t lcnum = host_readw(&rbuf[0]);
			i+=2;
			Bitu lcpos=0;
			for (;i<data_len;) {
				if (fread(rbuf, sizeof(uint8_t), 1,
				          tempfile.get()) != 1) {
					break;
				}
				i++;
				if (((char)rbuf[0])==',') break;
				lng_codes[lcpos++]=(char)rbuf[0];
			}
			lng_codes[lcpos]=0;
			if (strcasecmp(lng_codes, layout_id)==0) {
				// language ID found in file, return file position
				return cur_pos;
			}
			if (first_id_only) break;
			if (lcnum) {
				sprintf(&lng_codes[lcpos],"%d",lcnum);
				if (strcasecmp(lng_codes, layout_id)==0) {
					// language ID found in file, return file position
					return cur_pos;
				}
			}
		}
		if (fseek(tempfile.get(), cur_pos + 3 + len, SEEK_SET) != 0) {
			LOG_ERR("LAYOUT: could not seek to byte %d in keyboard layout file '%s': %s",
			        cur_pos + 3 + len, kcl_file_name, strerror(errno));
			return 0;
		}
	}

	return 0;
}

static uint32_t read_kcl_data(const std::vector<uint8_t> &kcl_data,
                              const char *layout_id,
                              const bool first_id_only)
{
	// check ID-bytes
	if ((kcl_data[0]!=0x4b) || (kcl_data[1]!=0x43) || (kcl_data[2]!=0x46)) {
		return 0;
	}

	uint32_t dpos=7+kcl_data[6];

	for (;;) {
		if (dpos + 5 > kcl_data.size())
			break;
		uint32_t cur_pos=dpos;
		uint16_t len=host_readw(&kcl_data[dpos]);
		uint8_t data_len=kcl_data[dpos+2];
		dpos+=5;

		char lng_codes[258];
		// get all language codes for this layout
		for (Bitu i=0; i<data_len;) {
			uint16_t lcnum=host_readw(&kcl_data[dpos-2]);
			i+=2;
			Bitu lcpos=0;
			for (;i<data_len;) {
				if (dpos + 1 > kcl_data.size())
					break;
				char lc=(char)kcl_data[dpos];
				dpos++;
				i++;
				if (lc==',') break;
				lng_codes[lcpos++]=lc;
			}
			lng_codes[lcpos]=0;
			if (strcasecmp(lng_codes, layout_id)==0) {
				// language ID found, return position
				return cur_pos;
			}
			if (first_id_only) break;
			if (lcnum) {
				sprintf(&lng_codes[lcpos],"%d",lcnum);
				if (strcasecmp(lng_codes, layout_id)==0) {
					// language ID found, return position
					return cur_pos;
				}
			}
			dpos+=2;
		}
		dpos=cur_pos+3+len;
	}
	return 0;
}

Bitu keyboard_layout::read_keyboard_file(const char* keyboard_file_name, int32_t specific_layout, int32_t requested_codepage) {
	this->reset();

	if (specific_layout == -1)
		safe_strcpy(current_keyboard_file_name, keyboard_file_name);
	if (!strcmp(keyboard_file_name,"none")) return KEYB_NOERROR;

	static uint8_t read_buf[65535];
	uint32_t read_buf_size, read_buf_pos, bytes_read;
	uint32_t start_pos=5;

	char nbuf[512];
	read_buf_size = 0;
	sprintf(nbuf, "%s.kl", keyboard_file_name);
	auto tempfile = OpenDosboxFile(nbuf);
	if (tempfile==NULL) {
		// try keyboard layout libraries next
		auto try_file = [&](const char *file, const bool first_id_only) {
			if (!(start_pos = read_kcl_file(file, keyboard_file_name,
			                                first_id_only)))
				return false;
			tempfile = OpenDosboxFile(file);
			return true;
		};
		auto try_builtin = [&](const uint8_t i, const bool first_id_only) {
			if (!(start_pos = read_kcl_data(BLOB_KEYBOARD_SYS[i],
			                                keyboard_file_name,
			                                first_id_only)))
				return false;
			assert(read_buf_size == 0);
			for (Bitu ct = start_pos + 2;
			     ct < BLOB_KEYBOARD_SYS[i].size(); ct++)
				read_buf[read_buf_size++] = BLOB_KEYBOARD_SYS[i][ct];
			return true;
		};
		if (!try_file("keyboard.sys", true) &&
		    !try_file("keybrd2.sys", true) &&
		    !try_file("keybrd3.sys", true) && !try_file("keybrd4.sys", true) &&
		    !try_file("keyboard.sys", false) &&
		    !try_file("keybrd2.sys", false) &&
		    !try_file("keybrd3.sys", false) &&
		    !try_file("keybrd4.sys", false) && !try_builtin(0, true) &&
		    !try_builtin(1, true) && !try_builtin(2, true) &&
		    !try_builtin(3, true) && !try_builtin(0, false) &&
		    !try_builtin(1, false) && !try_builtin(2, false) &&
		    !try_builtin(3, false)) {
			LOG(LOG_BIOS, LOG_ERROR)
			("Keyboard layout file %s not found", keyboard_file_name);
			return KEYB_FILENOTFOUND;
		}
		if (tempfile) {
			const auto seek_pos = start_pos + 2;
			if (fseek(tempfile.get(), seek_pos, SEEK_SET) != 0) {
				LOG_WARNING("LAYOUT: could not seek to byte %d in keyboard layout file '%s': %s",
				            seek_pos, keyboard_file_name, strerror(errno));
				return KEYB_INVALIDFILE;
			}
			read_buf_size = (uint32_t)fread(read_buf, sizeof(uint8_t),
			                              65535, tempfile.get());
		}
		start_pos=0;
	} else {
		// check ID-bytes of file
		uint32_t dr = (uint32_t)fread(read_buf, sizeof(uint8_t), 4,
		                          tempfile.get());
		if ((dr<4) || (read_buf[0]!=0x4b) || (read_buf[1]!=0x4c) || (read_buf[2]!=0x46)) {
			LOG(LOG_BIOS, LOG_ERROR)
			("Invalid keyboard layout file %s", keyboard_file_name);
			return KEYB_INVALIDFILE;
		}

		fseek(tempfile.get(), 0, SEEK_SET);
		read_buf_size = (uint32_t)fread(read_buf, sizeof(uint8_t), 65535,
		                              tempfile.get());
	}

	const auto data_len = read_buf[start_pos++];
	assert(data_len < UINT8_MAX);
	static_assert(UINT8_MAX < sizeof(read_buf), "read_buf too small");

	language_codes=new char*[data_len];
	language_code_count=0;
	// get all language codes for this layout
	for (Bitu i=0; i<data_len;) {
		language_codes[language_code_count]=new char[256];
		i+=2;
		Bitu lcpos=0;
		for (;i<data_len;) {
			assert(start_pos + i < sizeof(read_buf));
			char lcode=char(read_buf[start_pos+i]);
			i++;
			if (lcode==',') break;
			language_codes[language_code_count][lcpos++]=lcode;
		}
		language_codes[language_code_count][lcpos]=0;
		language_code_count++;
	}

	start_pos+=data_len;		// start_pos==absolute position of KeybCB block

	assert(start_pos < sizeof(read_buf));
	const auto submappings = read_buf[start_pos];

	assert(start_pos + 1 < sizeof(read_buf));
	additional_planes = read_buf[start_pos + 1];

	// four pages always occupied by normal,shift,flags,commandbits
	if (additional_planes>(layout_pages-4)) additional_planes=(layout_pages-4);

	// seek to plane descriptor
	read_buf_pos = start_pos + 0x14 + submappings * 8;

	assert(read_buf_pos < sizeof(read_buf));

	for (auto i = 0; i < additional_planes; ++i) {
		auto &layout = current_layout_planes[i];
		for (auto p_flags : {
		             &layout.required_flags,
		             &layout.forbidden_flags,
		             &layout.required_userflags,
		             &layout.forbidden_userflags,
		     }) {
			assert(read_buf_pos < sizeof(read_buf));
			*p_flags = host_readw(&read_buf[read_buf_pos]);
			read_buf_pos += 2;
			if (p_flags == &layout.required_flags) {
				used_lock_modifiers |= ((*p_flags) & 0x70);
			}
		}
	}

	bool found_matching_layout=false;
	
	// check all submappings and use them if general submapping or same codepage submapping
	for (uint16_t sub_map=0; (sub_map<submappings) && (!found_matching_layout); sub_map++) {
		uint16_t submap_cp, table_offset;

		if ((sub_map!=0) && (specific_layout!=-1)) sub_map=(uint16_t)(specific_layout&0xffff);

		// read codepage of submapping
		submap_cp=host_readw(&read_buf[start_pos+0x14+sub_map*8]);
		if ((submap_cp!=0) && (submap_cp!=requested_codepage) && (specific_layout==-1))
			continue;		// skip nonfitting submappings

		if (submap_cp==requested_codepage) found_matching_layout=true;

		// get table offset
		table_offset=host_readw(&read_buf[start_pos+0x18+sub_map*8]);
		diacritics_entries=0;
		if (table_offset!=0) {
			// process table
			uint16_t i,j;
			for (i=0; i<2048;) {
				if (read_buf[start_pos+table_offset+i]==0) break;	// end of table
				diacritics_entries++;
				i+=read_buf[start_pos+table_offset+i+1]*2+2;
			}
			// copy diacritics table
			for (j=0; j<=i; j++) diacritics[j]=read_buf[start_pos+table_offset+j];
		}


		// get table offset
		table_offset=host_readw(&read_buf[start_pos+0x16+sub_map*8]);
		if (table_offset==0) continue;	// non-present table

		read_buf_pos=start_pos+table_offset;

		bytes_read=read_buf_size-read_buf_pos;

		// process submapping table
		for (uint32_t i=0; i<bytes_read;) {
			uint8_t scan=read_buf[read_buf_pos++];
			if (scan==0) break;
			uint8_t scan_length=(read_buf[read_buf_pos]&7)+1;		// length of data struct
			read_buf_pos+=2;
			i+=3;

			assert(scan_length > 0);
			if ((scan & 0x7f) <= MAX_SCAN_CODE) {
				// add all available mappings
				for (uint16_t addmap=0; addmap<scan_length; addmap++) {
					if (addmap>additional_planes+2) break;
					Bitu charptr=read_buf_pos+addmap*((read_buf[read_buf_pos-2]&0x80)?2:1);
					uint16_t kchar=read_buf[charptr];

					if (kchar!=0) {		// key remapped
						if (read_buf[read_buf_pos-2]&0x80) kchar|=read_buf[charptr+1]<<8;	// scancode/char pair
						// overwrite mapping
						current_layout[scan*layout_pages+addmap]=kchar;
						// clear command bit
						current_layout[scan*layout_pages+layout_pages-2]&=~(1<<addmap);
						// add command bit
						current_layout[scan*layout_pages+layout_pages-2]|=(read_buf[read_buf_pos-1] & (1<<addmap));
					}
				}

				// calculate max length of entries, taking into account old number of entries
				uint8_t new_flags=current_layout[scan*layout_pages+layout_pages-1]&0x7;
				if ((read_buf[read_buf_pos-2]&0x7) > new_flags) new_flags = read_buf[read_buf_pos-2]&0x7;

				// merge flag bits in as well
				new_flags |= (read_buf[read_buf_pos-2] | current_layout[scan*layout_pages+layout_pages-1]) & 0xf0;

				current_layout[scan*layout_pages+layout_pages-1]=new_flags;
				if (read_buf[read_buf_pos-2]&0x80) scan_length*=2;		// granularity flag (S)
			}
			i+=scan_length;		// advance pointer
			read_buf_pos+=scan_length;
		}
		if (specific_layout==sub_map) break;
	}

	if (found_matching_layout) {
		if (specific_layout==-1) LOG(LOG_BIOS,LOG_NORMAL)("Keyboard layout %s successfully loaded",keyboard_file_name);
		else LOG(LOG_BIOS,LOG_NORMAL)("Keyboard layout %s (%i) successfully loaded",keyboard_file_name,specific_layout);
		this->use_foreign_layout=true;
		return KEYB_NOERROR;
	}

	LOG(LOG_BIOS,LOG_ERROR)("No matching keyboard layout found in %s",keyboard_file_name);

	// reset layout data (might have been changed by general layout)
	this->reset();

	return KEYB_LAYOUTNOTFOUND;
}

bool keyboard_layout::layout_key(Bitu key, uint8_t flags1, uint8_t flags2, uint8_t flags3) {
	if (key>MAX_SCAN_CODE) return false;
	if (!this->use_foreign_layout) return false;

	bool is_special_pair=(current_layout[key*layout_pages+layout_pages-1] & 0x80)==0x80;

	if ((((flags1&used_lock_modifiers)&0x7c)==0) && ((flags3&2)==0)) {
		// check if shift/caps is active:
		// (left_shift OR right_shift) XOR (key_affected_by_caps AND caps_locked)
		if ((((flags1&2)>>1) | (flags1&1)) ^ (((current_layout[key*layout_pages+layout_pages-1] & 0x40) & (flags1 & 0x40))>>6)) {
			// shift plane
			if (current_layout[key*layout_pages+1]!=0) {
				// check if command-bit is set for shift plane
				bool is_command=(current_layout[key*layout_pages+layout_pages-2]&2)!=0;
				if (this->map_key(key, current_layout[key*layout_pages+1],
					is_command, is_special_pair)) return true;
			}
		} else {
			// normal plane
			if (current_layout[key*layout_pages]!=0) {
				// check if command-bit is set for normal plane
				bool is_command=(current_layout[key*layout_pages+layout_pages-2]&1)!=0;
				if (this->map_key(key, current_layout[key*layout_pages],
					is_command, is_special_pair)) return true;
			}
		}
	}

	// calculate current flags
	uint16_t current_flags=(flags1&0x7f) | (((flags2&3) | (flags3&0xc))<<8);
	if (flags1&3) current_flags|=0x4000;	// either shift key active
	if (flags3&2) current_flags|=0x1000;	// e0 prefixed

	// check all planes if flags fit
	for (uint16_t cplane=0; cplane<additional_planes; cplane++) {
		uint16_t req_flags=current_layout_planes[cplane].required_flags;
		uint16_t req_userflags=current_layout_planes[cplane].required_userflags;
		// test flags
		if (((current_flags & req_flags)==req_flags) &&
			((user_keys & req_userflags)==req_userflags) &&
			((current_flags & current_layout_planes[cplane].forbidden_flags)==0) &&
			((user_keys & current_layout_planes[cplane].forbidden_userflags)==0)) {
				// remap key
				if (current_layout[key*layout_pages+2+cplane]!=0) {
					// check if command-bit is set for this plane
					bool is_command=((current_layout[key*layout_pages+layout_pages-2]>>(cplane+2))&1)!=0;
					if (this->map_key(key, current_layout[key*layout_pages+2+cplane],
						is_command, is_special_pair)) return true;
				} else break;	// abort plane checking
			}
	}

	if (diacritics_character>0) {
		// ignore state-changing keys
		switch(key) {
			case 0x1d:			/* Ctrl Pressed */
			case 0x2a:			/* Left Shift Pressed */
			case 0x36:			/* Right Shift Pressed */
			case 0x38:			/* Alt Pressed */
			case 0x3a:			/* Caps Lock */
			case 0x45:			/* Num Lock */
			case 0x46:			/* Scroll Lock */
				break;
			default:
			        if (diacritics_character >= diacritics_entries + 200) {
				        diacritics_character=0;
					return true;
			        }
			        uint16_t diacritics_start=0;
				// search start of subtable
				for (uint16_t i=0; i<diacritics_character-200; i++)
					diacritics_start+=diacritics[diacritics_start+1]*2+2;

				BIOS_AddKeyToBuffer((uint16_t)(key<<8) | diacritics[diacritics_start]);
				diacritics_character=0;
		}
	}

	return false;
}

bool keyboard_layout::map_key(Bitu key, uint16_t layouted_key, bool is_command, bool is_keypair) {
	if (is_command) {
		uint8_t key_command=(uint8_t)(layouted_key&0xff);
		// check if diacritics-command
		if ((key_command>=200) && (key_command<235)) {
			// diacritics command
			diacritics_character=key_command;
			if (diacritics_character >= diacritics_entries + 200)
				diacritics_character = 0;
			return true;
		} else if ((key_command>=120) && (key_command<140)) {
			// switch layout command
			this->read_keyboard_file(key_command-119);
			return true;
		} else if ((key_command>=180) && (key_command<188)) {
			// switch user key off
			user_keys&=~(1<<(key_command-180));
			return true;
		} else if ((key_command>=188) && (key_command<196)) {
			// switch user key on
			user_keys|=(1<<(key_command-188));
			return true;
		} else if (key_command==160) return true;	// nop command
	} else {
		// non-command
		if (diacritics_character>0) {
			if (diacritics_character-200>=diacritics_entries) diacritics_character = 0;
			else {
				uint16_t diacritics_start=0;
				// search start of subtable
				for (uint16_t i=0; i<diacritics_character-200; i++)
					diacritics_start+=diacritics[diacritics_start+1]*2+2;

				uint8_t diacritics_length=diacritics[diacritics_start+1];
				diacritics_start+=2;
				diacritics_character=0;	// reset

				// search scancode
				for (uint16_t i=0; i<diacritics_length; i++) {
					if (diacritics[diacritics_start+i*2]==(layouted_key&0xff)) {
						// add diacritics to keybuf
						BIOS_AddKeyToBuffer((uint16_t)(key<<8) | diacritics[diacritics_start+i*2+1]);
						return true;
					}
				}
				// add standard-diacritics to keybuf
				BIOS_AddKeyToBuffer((uint16_t)(key<<8) | diacritics[diacritics_start-2]);
			}
		}

		// add remapped key to keybuf
		if (is_keypair) BIOS_AddKeyToBuffer(layouted_key);
		else BIOS_AddKeyToBuffer((uint16_t)(key<<8) | (layouted_key&0xff));

		return true;
	}
	return false;
}

uint16_t keyboard_layout::extract_codepage(const char* keyboard_file_name) {
	if (!strcmp(keyboard_file_name,"none")) return 437;

	uint32_t read_buf_size;
	static uint8_t read_buf[65535];
	uint32_t start_pos=5;

	char nbuf[512];
	sprintf(nbuf, "%s.kl", keyboard_file_name);
	auto tempfile = OpenDosboxFile(nbuf);
	if (!tempfile) {
		// try keyboard layout libraries next
		auto try_file = [&](const char *file, const bool first_id_only) {
			if (!(start_pos = read_kcl_file(file, keyboard_file_name,
			                                first_id_only)))
				return false;
			tempfile = OpenDosboxFile(file);
			return true;
		};
		auto try_builtin = [&](const uint8_t i, const bool first_id_only) {
			if (!(start_pos = read_kcl_data(BLOB_KEYBOARD_SYS[i],
			                                keyboard_file_name,
			                                first_id_only)))
				return false;
			read_buf_size = 0;
			for (Bitu ct = start_pos + 2;
			     ct < BLOB_KEYBOARD_SYS[i].size(); ct++)
				read_buf[read_buf_size++] = BLOB_KEYBOARD_SYS[i][ct];
			return true;
		};
		if (!try_file("keyboard.sys", true) &&
		    !try_file("keybrd2.sys", true) &&
		    !try_file("keybrd3.sys", true) && !try_file("keybrd4.sys", true) &&
		    !try_file("keyboard.sys", false) &&
		    !try_file("keybrd2.sys", false) &&
		    !try_file("keybrd3.sys", false) &&
		    !try_file("keybrd4.sys", false) && !try_builtin(0, true) &&
		    !try_builtin(1, true) && !try_builtin(2, true) &&
		    !try_builtin(3, true) && !try_builtin(0, false) &&
		    !try_builtin(1, false) && !try_builtin(2, false) &&
		    !try_builtin(3, false)) {
			LOG(LOG_BIOS, LOG_ERROR)
			("Keyboard layout file %s not found", keyboard_file_name);
			return 437;
		}
		if (tempfile) {
			fseek(tempfile.get(), start_pos + 2, SEEK_SET);
			read_buf_size = (uint32_t)fread(read_buf, sizeof(uint8_t),
			                              65535, tempfile.get());
		}
		start_pos=0;
	} else {
		// check ID-bytes of file
		uint32_t dr = (uint32_t)fread(read_buf, sizeof(uint8_t), 4,
		                          tempfile.get());
		if ((dr<4) || (read_buf[0]!=0x4b) || (read_buf[1]!=0x4c) || (read_buf[2]!=0x46)) {
			LOG(LOG_BIOS, LOG_ERROR)
			("Invalid keyboard layout file %s", keyboard_file_name);
			return 437;
		}

		fseek(tempfile.get(), 0, SEEK_SET);
		read_buf_size = (uint32_t)fread(read_buf, sizeof(uint8_t), 65535,
		                              tempfile.get());
	}

	uint8_t data_len,submappings;
	data_len=read_buf[start_pos++];

	start_pos+=data_len;		// start_pos==absolute position of KeybCB block

	assert(start_pos < sizeof(read_buf));
	submappings=read_buf[start_pos];

	// Make sure the submappings value won't let us read beyond the end of
	// the buffer
	if (submappings >= ceil_udivide(sizeof(read_buf) - start_pos - 0x14, 8u)) {
		LOG(LOG_BIOS, LOG_ERROR)
		("Keyboard layout file %s is corrupt", keyboard_file_name);
		return 437;
	}

	// check all submappings and use them if general submapping or same
	// codepage submapping
	for (uint16_t sub_map=0; (sub_map<submappings); sub_map++) {
		uint16_t submap_cp;

		// read codepage of submapping
		submap_cp=host_readw(&read_buf[start_pos+0x14+sub_map*8]);

		if (submap_cp!=0) return submap_cp;
	}
	return 437;
}

int8_t keyboard_layout::get_CPX_file_id(uint32_t codepage_id)
{
	// reference:
	// https://gitlab.com/FreeDOS/base/cpidos/-/blob/master/DOC/CPIDOS/CODEPAGE.TXT
	switch (codepage_id) {
	case 437:
	case 850:
	case 852:
	case 853:
	case 857:
	case 858: return 0; // EGA.CPX
	case 775:
	case 859:
	case 1116:
	case 1117:
	case 1118:
	case 1119: return 1; // EGA2.CPX
	case 771:
	case 772:
	case 808:
	case 855:
	case 866:
	case 872: return 2; // EGA3.CPX
	case 848:
	case 849:
	case 1125:
	case 1131:
	case 3012:
	case 30010: return 3; // EGA4.CPX
	case 113:
	case 737:
	case 851:
	case 869: return 4; // EGA5.CPX
	case 899:
	case 30008:
	case 58210:
	case 59829:
	case 60258:
	case 60853: return 5; // EGA6.CPX
	case 30011:
	case 30013:
	case 30014:
	case 30017:
	case 30018:
	case 30019: return 6; // EGA7.CPX
	case 770:
	case 773:
	case 774:
	case 777:
	case 778: return 7; // EGA8.CPX
	case 860:
	case 861:
	case 863:
	case 865:
	case 867: return 8; // EGA9.CPX
	case 667:
	case 668:
	case 790:
	case 991:
	case 3845: return 9; // EGA10.CPX
	case 30000:
	case 30001:
	case 30004:
	case 30007:
	case 30009: return 10; // EGA11.CPX
	case 30003:
	case 30029:
	case 30030:
	case 58335: return 11; // EGA12.CPX
	case 895:
	case 30002:
	case 58152:
	case 59234:
	case 62306: return 12; // EGA13.CPX
	case 30006:
	case 30012:
	case 30015:
	case 30016:
	case 30020:
	case 30021: return 13; // EGA14.CPX
	case 30023:
	case 30024:
	case 30025:
	case 30026:
	case 30027:
	case 30028: return 14; // EGA15.CPX
	case 3021:
	case 30005:
	case 30022:
	case 30031:
	case 30032: return 15; // EGA16.CPX
	case 862:
	case 864:
	case 30034:
	case 30033:
	case 30039:
	case 30040: return 16; // EGA17.CPX
	case 856:
	case 3846:
	case 3848: return 17; // EGA18.CPX
	default: return -1;   // none
	}
}

Bitu keyboard_layout::read_codepage_file(const char *codepage_file_name,
                                         int32_t codepage_id)
{
	char cp_filename[512];
	safe_strcpy(cp_filename, codepage_file_name);
	if (!strcmp(cp_filename,"none")) return KEYB_NOERROR;

	if (codepage_id==dos.loaded_codepage) return KEYB_NOERROR;

	if (!strcmp(cp_filename,"auto")) {
		// select matching .cpi-file for specified codepage
		const auto i = get_CPX_file_id(codepage_id);
		if (i < 0) {
			LOG_MSG("No matching cpi file for codepage %i", codepage_id);
			return KEYB_INVALIDCPFILE;
		}
		sprintf(cp_filename, "%s", FILE_EGA_CPX[i]);
	}

	char nbuf[520];
	sprintf(nbuf, "Z:\\CPI\\%s", cp_filename);
	auto tempfile = OpenDosboxFile(nbuf);
	if (tempfile==NULL) {
		size_t strsz=strlen(nbuf);
		if (strsz) {
			char plc=(char)toupper(*reinterpret_cast<unsigned char*>(&nbuf[strsz-1]));
			if (plc=='I') {
				// try CPX-extension as well
				nbuf[strsz-1]='X';
				tempfile=OpenDosboxFile(nbuf);
			} else if (plc=='X') {
				// try CPI-extension as well
				nbuf[strsz-1]='I';
				tempfile=OpenDosboxFile(nbuf);
			}
		}
	}

	std::array<uint8_t, UINT16_MAX + 1> cpi_buf;
	constexpr size_t cpi_unit_size = sizeof(cpi_buf[0]);

	size_t cpi_buf_size = 0;
	size_t size_of_cpxdata = 0;
	bool upxfound = false;
	size_t found_at_pos = 5;
	if (tempfile==NULL) {
		// check if build-in codepage is available
		const auto i = get_CPX_file_id(codepage_id);
		if (i < 0)
			return KEYB_INVALIDCPFILE;
		cpi_buf_size = BLOB_EGA_CPX[i].size();
		for (size_t bct = 0; bct < cpi_buf_size; bct++)
			cpi_buf[bct] = BLOB_EGA_CPX[i][bct];
		upxfound = true;
		found_at_pos=0x29;
		size_of_cpxdata=cpi_buf_size;
	} else {
		constexpr auto bytes_to_detect_upx = 5;
		const auto dr = fread(cpi_buf.data(), cpi_unit_size, bytes_to_detect_upx, tempfile.get());
		// check if file is valid
		if (dr<5) {
			LOG(LOG_BIOS,LOG_ERROR)("Codepage file %s invalid",cp_filename);
			return KEYB_INVALIDCPFILE;
		}
		// check if non-compressed cpi file
		if ((cpi_buf[0]!=0xff) || (cpi_buf[1]!=0x46) || (cpi_buf[2]!=0x4f) || 
			(cpi_buf[3]!=0x4e) || (cpi_buf[4]!=0x54)) {
			// check if dr-dos custom cpi file
			if ((cpi_buf[0]==0x7f) && (cpi_buf[1]!=0x44) && (cpi_buf[2]!=0x52) && 
				(cpi_buf[3]!=0x46) && (cpi_buf[4]!=0x5f)) {
				LOG(LOG_BIOS,LOG_ERROR)("Codepage file %s has unsupported DR-DOS format",cp_filename);
				return KEYB_INVALIDCPFILE;
			}

			// Read enough data to scan for UPX's identifier and version
			const auto scan_size = 100;
			assert(scan_size <= cpi_buf.size());
			if (fread(cpi_buf.data(), cpi_unit_size, scan_size, tempfile.get()) != scan_size) {
				LOG_WARNING("CODEPAGE: File %s is too small, could not read initial %d bytes",
				            cp_filename, scan_size + ds);
				return KEYB_INVALIDCPFILE;
			}
			// Scan for the UPX identifier
			const auto upx_id = sv{"UPX!"};
			const auto scan_buf = sv{reinterpret_cast<char *>(cpi_buf.data()), scan_size};
			const auto upx_id_pos = scan_buf.find(upx_id);

			// did we find the UPX identifier?
			upxfound = upx_id_pos != scan_buf.npos;
			if (!upxfound) {
				LOG_WARNING("CODEPAGE: File %s is invalid, could not find the UPX identifier",
				            cp_filename);
				return KEYB_INVALIDCPFILE;
			}
			// The IPX version byte comes after the identifier pattern
			const auto upx_ver_pos = upx_id_pos + upx_id.length();
			const auto upx_ver = cpi_buf[upx_ver_pos];

			// Can we handle this version?
			constexpr uint8_t upx_min_ver = 10;
			if (upx_ver < upx_min_ver) {
				LOG_WARNING("CODEPAGE: File %s is packed with UPX version %u, but %u+ is needed",
				            cp_filename, upx_ver, upx_min_ver);
				return KEYB_INVALIDCPFILE;
			}
			// The next data comes after the version (used for decompression below)
			found_at_pos += upx_ver_pos + sizeof(upx_ver);

			// Read the entire compressed CPX-file
			fseek(tempfile.get(), 0, SEEK_SET);
			size_of_cpxdata = fread(cpi_buf.data(), cpi_unit_size, cpi_buf.size(), tempfile.get());
		} else {
			// standard uncompressed cpi-file
			fseek(tempfile.get(), 0, SEEK_SET);
			cpi_buf_size = fread(cpi_buf.data(), cpi_unit_size, cpi_buf.size(), tempfile.get());
		}
	}

	if (upxfound) {
		if (size_of_cpxdata>0xfe00) E_Exit("Size of cpx-compressed data too big");

		found_at_pos+=19;
		// prepare for direct decompression
		cpi_buf[found_at_pos]=0xcb;

		uint16_t seg=0;
		uint16_t size=0x1500;
		if (!DOS_AllocateMemory(&seg,&size)) E_Exit("Not enough free low memory to unpack data");

		const auto dos_segment = static_cast<uint32_t>((seg << 4) + 0x100);
		assert(size_of_cpxdata <= cpi_buf.size());
		MEM_BlockWrite(dos_segment, cpi_buf.data(), size_of_cpxdata);

		// setup segments
		uint16_t save_ds=SegValue(ds);
		uint16_t save_es=SegValue(es);
		uint16_t save_ss=SegValue(ss);
		uint32_t save_esp=reg_esp;
		SegSet16(ds,seg);
		SegSet16(es,seg);
		SegSet16(ss,seg+0x1000);
		reg_esp=0xfffe;

		// let UPX unpack the file
		CALLBACK_RunRealFar(seg,0x100);

		SegSet16(ds,save_ds);
		SegSet16(es,save_es);
		SegSet16(ss,save_ss);
		reg_esp=save_esp;

		// get unpacked content
		MEM_BlockRead(dos_segment, cpi_buf.data(), cpi_buf.size());
		cpi_buf_size=65536;

		DOS_FreeMemory(seg);
	}

	constexpr auto data_start_index = 0x13;
	static_assert(data_start_index < cpi_buf.size());
	auto start_pos = host_readd(&cpi_buf[data_start_index]);

	// Internally unpacking some UPX code-page files can result in unparseable data
	if (start_pos >= cpi_buf_size) {
		LOG_WARNING("KEYBOARD: Could not parse %scode-data from: %s",
		            (upxfound) ? "UPX-unpacked " : "", cp_filename);

		LOG(LOG_BIOS, LOG_ERROR)
		("Code-page file %s invalid start_pos=%u", cp_filename, start_pos);
		return KEYB_INVALIDCPFILE;
	}

	const auto number_of_codepages = host_readw(&cpi_buf.at(start_pos));
	start_pos+=4;

	// search if codepage is provided by file
	for (uint16_t test_codepage = 0; test_codepage < number_of_codepages; test_codepage++) {
		// device type can be display/printer (only the first is supported)
		const auto device_type = host_readw(&cpi_buf.at(start_pos + 0x04));
		const auto font_codepage = host_readw(&cpi_buf.at(start_pos + 0x0e));
		const auto font_data_header_pt = host_readd(&cpi_buf.at(start_pos + 0x16));
		const auto font_type = host_readw(&cpi_buf.at(font_data_header_pt));

		if ((device_type==0x0001) && (font_type==0x0001) && (font_codepage==codepage_id)) {
			// valid/matching codepage found

			const auto number_of_fonts = host_readw(&cpi_buf.at(font_data_header_pt + 0x02));
			// const uint16_t font_data_length = host_readw(&cpi_buf[font_data_header_pt + 0x04]);

			auto font_data_start = font_data_header_pt + 0x06;

			// load all fonts if possible
			bool font_changed = false;
			for (uint16_t current_font = 0; current_font < number_of_fonts; ++current_font) {
				const auto font_height = cpi_buf.at(font_data_start);
				font_data_start += 6;
				if (font_height == 0x10) {
					// 16x8 font
					const auto font16pt = Real2Phys(int10.rom.font_16);
					for (uint16_t i = 0; i < 256 * 16; ++i) {
						phys_writeb(font16pt + i, cpi_buf.at(font_data_start + i));
					}
					// terminate alternate list to prevent loading
					phys_writeb(Real2Phys(int10.rom.font_16_alternate),0);
					font_changed=true;
				} else if (font_height == 0x0e) {
					// 14x8 font
					const auto font14pt = Real2Phys(int10.rom.font_14);
					for (uint16_t i = 0; i < 256 * 14; ++i) {
						phys_writeb(font14pt + i, cpi_buf.at(font_data_start + i));
					}
					// terminate alternate list to prevent loading
					phys_writeb(Real2Phys(int10.rom.font_14_alternate),0);
					font_changed=true;
				} else if (font_height == 0x08) {
					// 8x8 fonts
					auto font8pt = Real2Phys(int10.rom.font_8_first);
					for (uint16_t i = 0; i < 128 * 8; ++i) {
						phys_writeb(font8pt + i, cpi_buf.at(font_data_start + i));
					}
					font8pt=Real2Phys(int10.rom.font_8_second);
					for (uint16_t i = 0; i < 128 * 8; ++i) {
						phys_writeb(font8pt + i,
						            cpi_buf.at(font_data_start + i + 128 * 8));
					}
					font_changed=true;
				}
				font_data_start+=font_height*256;
			}

			LOG(LOG_BIOS,LOG_NORMAL)("Codepage %i successfully loaded",codepage_id);

			// set codepage entries
			dos.loaded_codepage=(uint16_t)(codepage_id&0xffff);

			// update font if necessary
			if (font_changed && (CurMode->type==M_TEXT) && (IS_EGAVGA_ARCH)) {
				INT10_ReloadFont();
			}
			INT10_SetupRomMemoryChecksum();

			return KEYB_NOERROR;
		}

		start_pos = host_readd(&cpi_buf.at(start_pos));
		start_pos+=2;
	}

	LOG(LOG_BIOS,LOG_ERROR)("Codepage %i not found",codepage_id);

	return KEYB_INVALIDCPFILE;
}

Bitu keyboard_layout::switch_keyboard_layout(const char* new_layout, keyboard_layout*& created_layout, int32_t& tried_cp) {
	if (strncasecmp(new_layout,"US",2)) {
		// switch to a foreign layout
		char tbuf[256];
		safe_strcpy(tbuf, new_layout);
		size_t newlen=strlen(tbuf);

		bool language_code_found=false;
		// check if language code is present in loaded foreign layout
		for (Bitu i=0; i<language_code_count; i++) {
			if (!strncasecmp(tbuf,language_codes[i],newlen)) {
				language_code_found=true;
				break;
			}
		}

		if (language_code_found) {
			if (!this->use_foreign_layout) {
				// switch to foreign layout
				this->use_foreign_layout=true;
				diacritics_character=0;
				LOG(LOG_BIOS,LOG_NORMAL)("Switched to layout %s",tbuf);
			}
		} else {
			keyboard_layout * temp_layout=new keyboard_layout();
			Bitu req_codepage=temp_layout->extract_codepage(new_layout);
			tried_cp = req_codepage;
			Bitu kerrcode=temp_layout->read_keyboard_file(new_layout, req_codepage);
			if (kerrcode) {
				delete temp_layout;
				return kerrcode;
			}
			// ...else keyboard layout loaded successfully, change codepage accordingly
			kerrcode=temp_layout->read_codepage_file("auto", req_codepage);
			if (kerrcode) {
				delete temp_layout;
				return kerrcode;
			}
			// Everything went fine, switch to new layout
			created_layout=temp_layout;
		}
	} else if (this->use_foreign_layout) {
		// switch to the US layout
		this->use_foreign_layout=false;
		diacritics_character=0;
		LOG(LOG_BIOS,LOG_NORMAL)("Switched to US layout");
	}
	return KEYB_NOERROR;
}

void keyboard_layout::switch_foreign_layout() {
	this->use_foreign_layout=!this->use_foreign_layout;
	diacritics_character=0;
	if (this->use_foreign_layout) LOG(LOG_BIOS,LOG_NORMAL)("Switched to foreign layout");
	else LOG(LOG_BIOS,LOG_NORMAL)("Switched to US layout");
}

const char* keyboard_layout::get_layout_name() {
	// get layout name (language ID or NULL if default layout)
	if (use_foreign_layout) {
		if (strcmp(current_keyboard_file_name,"none") != 0) {
			return (const char*)&current_keyboard_file_name;
		}
	}
	return NULL;
}

const char* keyboard_layout::main_language_code() {
	if (language_codes) {
		return language_codes[0];
	}
	return NULL;
}


static keyboard_layout* loaded_layout=NULL;

#if 0
// Ctrl+Alt+F2 switches between foreign and US-layout using this function
static void switch_keyboard_layout(bool pressed) {
	if (!pressed)
		return;
	if (loaded_layout) loaded_layout->switch_foreign_layout();
}
#endif

// called by int9-handler
bool DOS_LayoutKey(Bitu key, uint8_t flags1, uint8_t flags2, uint8_t flags3) {
	if (loaded_layout) return loaded_layout->layout_key(key, flags1, flags2, flags3);
	else return false;
}

Bitu DOS_LoadKeyboardLayout(const char * layoutname, int32_t codepage, const char * codepagefile) {
	keyboard_layout * temp_layout=new keyboard_layout();
	// try to read the layout for the specified codepage
	Bitu kerrcode=temp_layout->read_keyboard_file(layoutname, codepage);
	if (kerrcode) {
		delete temp_layout;
		return kerrcode;
	}
	// ...else keyboard layout loaded successfully, change codepage accordingly
	kerrcode=temp_layout->read_codepage_file(codepagefile, codepage);
	if (kerrcode) {
		delete temp_layout;
		return kerrcode;
	}
	// Everything went fine, switch to new layout
	loaded_layout=temp_layout;
	return KEYB_NOERROR;
}

Bitu DOS_SwitchKeyboardLayout(const char* new_layout, int32_t& tried_cp) {
	if (loaded_layout) {
		keyboard_layout* changed_layout=NULL;
		Bitu ret_code=loaded_layout->switch_keyboard_layout(new_layout, changed_layout, tried_cp);
		if (changed_layout) {
			// Remove old layout, activate new layout
			delete loaded_layout;
			loaded_layout=changed_layout;
		}
		return ret_code;
	} else return 0xff;
}

// get currently loaded layout name (NULL if no layout is loaded)
const char* DOS_GetLoadedLayout(void) {
	if (loaded_layout) {
		return loaded_layout->get_layout_name();
	}
	return NULL;
}


class DOS_KeyboardLayout final : public Module_base {
public:
	DOS_KeyboardLayout(Section* configuration):Module_base(configuration){
		Section_prop * section=static_cast<Section_prop *>(configuration);
		dos.loaded_codepage=437;	// US codepage already initialized
		loaded_layout=new keyboard_layout();

		const char * layoutname=section->Get_string("keyboardlayout");

#if defined (WIN32)
		Bits wants_dos_codepage = -1;
		if (!strncmp(layoutname, "auto", 4)) {
			WORD cur_kb_layout = LOWORD(GetKeyboardLayout(0));
			WORD cur_kb_subID  = 0;
			char layoutID_string[KL_NAMELENGTH];
			if (GetKeyboardLayoutName(layoutID_string)) {
				if (safe_strlen(layoutID_string) == 8) {
					int cur_kb_layout_by_name = ConvHexWord((char*)&layoutID_string[4]);
					layoutID_string[4] = 0;
					int subID = ConvHexWord((char*)&layoutID_string[0]);
					if ((cur_kb_layout_by_name>0) && (cur_kb_layout_by_name<65536)) {
						// use layout ID extracted from the layout string
						cur_kb_layout = (WORD)cur_kb_layout_by_name;
					}
					if ((subID>=0) && (subID<100)) {
						// use sublanguage ID extracted from the layout string
						cur_kb_subID  = (WORD)subID;
					}
				}
			}
			// try to match emulated keyboard layout with host-keyboardlayout
			// codepage 437 (standard) is preferred
			switch (cur_kb_layout) {
/*				case 1026:
					layoutname = "bg241";
					break; */
				case 1029:
					layoutname = "cz243";
					break;
				case 1030:
					layoutname = "dk";
					break;
				case 1031:
					layoutname = "gr";
					wants_dos_codepage = 437;
					break;
				case 1033:
					// US
					return;
				case 1032:
					layoutname = "gk";
					break;
				case 1034:
					layoutname = "sp";
					wants_dos_codepage = 437;
					break;
				case 1035:
					layoutname = "su";
					wants_dos_codepage = 437;
					break;
				case 1036:
					layoutname = "fr";
					wants_dos_codepage = 437;
					break;
				case 1038:
					if (cur_kb_subID==1) layoutname = "hu";
					else layoutname = "hu208";
					break;
				case 1039:
					layoutname = "is161";
					break;
				case 1040:
					layoutname = "it";
					wants_dos_codepage = 437;
					break;
				case 1043:
					layoutname = "nl";
					wants_dos_codepage = 437;
					break;
				case 1044:
					layoutname = "no";
					break;
				case 1045:
					layoutname = "pl";
					break;
				case 1046:
					layoutname = "br";
					wants_dos_codepage = 437;
					break;
/*				case 1048:
					layoutname = "ro446";
					break; */
				case 1049:
					layoutname = "ru";
					wants_dos_codepage = 437;
					break;
				case 1050:
					layoutname = "hr";
					break;
				case 1051:
					layoutname = "sk";
					break;
/*				case 1052:
					layoutname = "sq448";
					break; */
				case 1053:
					layoutname = "sv";
					wants_dos_codepage = 437;
					break;
				case 1055:
					layoutname = "tr";
					break;
				case 1058:
					layoutname = "ur";
					wants_dos_codepage = 437;
					break;
				case 1059:
					layoutname = "bl";
					break;
				case 1060:
					layoutname = "si";
					break;
				case 1061:
					layoutname = "et";
					break;
/*				case 1062:
					layoutname = "lv";
					break; */
/*				case 1063:
					layoutname = "lt221";
					break; */
/*				case 1064:
					layoutname = "tj";
					break;
				case 1066:
					layoutname = "vi";
					break;
				case 1067:
					layoutname = "hy";
					break; */
				case 2055:
					layoutname = "sg";
					wants_dos_codepage = 437;
					break;
				case 2070:
					layoutname = "po";
					break;
				case 4108:
					layoutname = "sf";
					wants_dos_codepage = 437;
					break;
				default:
					break;
			}
		}
		// this condition may only occur on Windows
		if (wants_dos_codepage > 0) {
			if ((loaded_layout->read_codepage_file("auto", (Bitu)wants_dos_codepage)) == KEYB_NOERROR) {
				// preselected codepage was successfully loaded, so do nothing
			} else {
				// try to find a good codepage for the requested layout
				const auto req_codepage = loaded_layout->extract_codepage(layoutname);
				loaded_layout->read_codepage_file("auto", req_codepage);
			}
		}
#else
		// On non-Windows systems, alwaays find a good codepage for the requested layout
		const auto req_codepage = loaded_layout->extract_codepage(layoutname);
		loaded_layout->read_codepage_file("auto", req_codepage);
#endif

/*		if (strncmp(layoutname,"auto",4) && strncmp(layoutname,"none",4)) {
			LOG_MSG("Loading DOS keyboard layout %s ...",layoutname);
		} */
		if (loaded_layout->read_keyboard_file(layoutname, dos.loaded_codepage)) {
			if (strncmp(layoutname,"auto",4)) {
				LOG_ERR("Error loading keyboard layout %s",layoutname);
			}
		} else {
			const char* lcode = loaded_layout->main_language_code();
			if (lcode) {
				LOG_MSG("DOS keyboard layout loaded with main language code %s for layout %s",lcode,layoutname);
			}
		}
	}

	~DOS_KeyboardLayout(){
		if ((dos.loaded_codepage!=437) && (CurMode->type==M_TEXT)) {
			INT10_ReloadRomFonts();
			dos.loaded_codepage=437;	// US codepage
		}
		if (loaded_layout) {
			delete loaded_layout;
			loaded_layout=NULL;
		}
	}
};

static DOS_KeyboardLayout* test;

void DOS_KeyboardLayout_ShutDown(Section* /*sec*/) {
	delete test;	
}

const std::map<std::string, int> country_code_map {
	// reference: https://gitlab.com/FreeDOS/base/keyb_lay/-/blob/master/DOC/KEYB/LAYOUTS/LAYOUTS.TXT
	{"ar462",  COUNTRY::Arabic         },
	{"ar470",  COUNTRY::Arabic         },
	{"az",     COUNTRY::Azerbaijan     },
	{"ba",     COUNTRY::Bosnia         },
	{"be",     COUNTRY::Belgium        },
	{"bg",     COUNTRY::Bulgaria       }, // 101-key
	{"bg103",  COUNTRY::Bulgaria       }, // 101-key, Phonetic
	{"bg241",  COUNTRY::Bulgaria       }, // 102-key
	{"bl",     COUNTRY::Belarus        },
	{"bn",     COUNTRY::Benin          },
	{"br",     COUNTRY::Brazil         }, // ABNT layout
	{"br274",  COUNTRY::Brazil         }, // US layout
	{"bx",     COUNTRY::Belgium        }, // International
	{"by",     COUNTRY::Belarus        },
	{"ca",     COUNTRY::Candian_French }, // Standard
	{"ce",     COUNTRY::Russia         }, // Chechnya Standard
	{"ce443",  COUNTRY::Russia         }, // Chechnya Typewriter
	{"cg",     COUNTRY::Montenegro     },
	{"cf",     COUNTRY::Candian_French }, // Standard
	{"cf445",  COUNTRY::Candian_French }, // Dual-layer
	{"co",     COUNTRY::United_States  }, // Colemak
	{"cz",     COUNTRY::Czech_Slovak   }, // Czechia, QWERTY
	{"cz243",  COUNTRY::Czech_Slovak   }, // Czechia, Standard
	{"cz489",  COUNTRY::Czech_Slovak   }, // Czechia, Programmers
	{"de",     COUNTRY::Germany        }, // Standard
	{"dk",     COUNTRY::Denmark        },
	{"dv",     COUNTRY::United_States  }, // Dvorak
	{"ee",     COUNTRY::Estonia        },
	{"el",     COUNTRY::Greece         }, // 319
	{"es",     COUNTRY::Spain          },
	{"et",     COUNTRY::Estonia        },
	{"fi",     COUNTRY::Finland        },
	{"fo",     COUNTRY::Faeroe_Islands },
	{"fr",     COUNTRY::France         }, // Standard
	{"fx",     COUNTRY::France         }, // International
	{"gk",     COUNTRY::Greece         }, // 319
	{"gk220",  COUNTRY::Greece         }, // 220
	{"gk459",  COUNTRY::Greece         }, // 101-key
	{"gr",     COUNTRY::Germany        }, // Standard
	{"gr453",  COUNTRY::Germany        }, // Dual-layer
	{"hr",     COUNTRY::Croatia        },
	{"hu",     COUNTRY::Hungary        }, // 101-key
	{"hu208",  COUNTRY::Hungary        }, // 102-key
	{"hy",     COUNTRY::Armenia        },
	{"il",     COUNTRY::Israel         },
	{"is",     COUNTRY::Iceland        }, // 101-key
	{"is161",  COUNTRY::Iceland        }, // 102-key
	{"it",     COUNTRY::Italy          }, // Standard
	{"it142",  COUNTRY::Italy          }, // Comma on Numeric Pad
	{"ix",     COUNTRY::Italy          }, // International
	{"jp",     COUNTRY::Japan          },
	{"ka",     COUNTRY::Georgia        },
	{"kk",     COUNTRY::Kazakhstan     },
	{"kk476",  COUNTRY::Kazakhstan     },
	{"kx",     COUNTRY::United_Kingdom }, // International
	{"ky",     COUNTRY::Kyrgyzstan     },
	{"la",     COUNTRY::Latin_America  },
	{"lh",     COUNTRY::United_States  }, // Left-Hand Dvorak
	{"lt",     COUNTRY::Lithuania      }, // Baltic
	{"lt210",  COUNTRY::Lithuania      }, // 101-key, Programmers
	{"lt211",  COUNTRY::Lithuania      }, // AZERTY
	{"lt221",  COUNTRY::Lithuania      }, // Standard
	{"lt456",  COUNTRY::Lithuania      }, // Dual-layout
	{"lv",     COUNTRY::Latvia         }, // Standard
	{"lv455",  COUNTRY::Latvia         }, // Dual-layout
	{"ml",     COUNTRY::Malta          }, // UK-based
	{"mk",     COUNTRY::Macedonia      },
	{"mn",     COUNTRY::Mongolia       },
	{"mo",     COUNTRY::Mongolia       },
	{"mt",     COUNTRY::Malta          }, // UK-based
	{"mt103",  COUNTRY::Malta          }, // US-based
	{"ne",     COUNTRY::Niger          },
	{"ng",     COUNTRY::Nigeria        },
	{"nl",     COUNTRY::Netherlands    }, // 102-key
	{"no",     COUNTRY::Norway         },
	{"ph",     COUNTRY::Philippines    },
	{"pl",     COUNTRY::Poland         }, // 101-key, Programmers
	{"pl214",  COUNTRY::Poland         }, // 102-key
	{"po",     COUNTRY::Portugal       },
	{"px",     COUNTRY::Portugal       }, // International
	{"ro",     COUNTRY::Romania        }, // Standard
	{"ro446",  COUNTRY::Romania        }, // QWERTY
	{"rh",     COUNTRY::United_States  }, // Right-Hand Dvorak
	{"ru",     COUNTRY::Russia         }, // Standard
	{"ru443",  COUNTRY::Russia         }, // Typewriter
	{"rx",     COUNTRY::Russia         }, // Extended Standard
	{"rx443",  COUNTRY::Russia         }, // Extended Typewriter
	{"sd",     COUNTRY::Switzerland    }, // German
	{"sf",     COUNTRY::Switzerland    }, // French
	{"sg",     COUNTRY::Switzerland    }, // German
	{"si",     COUNTRY::Slovenia       },
	{"sk",     COUNTRY::Czech_Slovak   }, // Slovakia
	{"sp",     COUNTRY::Spain          },
	{"sq",     COUNTRY::Albania        }, // No-deadkeys
	{"sq448",  COUNTRY::Albania        }, // Deadkeys
	{"sr",     COUNTRY::Serbia         }, // Deadkey
	{"su",     COUNTRY::Finland        },
	{"sv",     COUNTRY::Sweden         },
	{"sx",     COUNTRY::Spain          }, // International
	{"tj",     COUNTRY::Tadjikistan    },
	{"tm",     COUNTRY::Turkmenistan   },
	{"tr",     COUNTRY::Turkey         }, // QWERTY
	{"tr440",  COUNTRY::Turkey         }, // Non-standard
	{"tt",     COUNTRY::Russia         }, // Tatarstan Standard
	{"tt443",  COUNTRY::Russia         }, // Tatarstan Typewriter
	{"ua",     COUNTRY::Ukraine        }, // 101-key
	{"uk",     COUNTRY::United_Kingdom }, // Standard
	{"uk168",  COUNTRY::United_Kingdom }, // Allternate
	{"ur",     COUNTRY::Ukraine        }, // 101-key
	{"ur465",  COUNTRY::Ukraine        }, // 101-key
	{"ur1996", COUNTRY::Ukraine        }, // 101-key
	{"ur2001", COUNTRY::Ukraine        }, // 102-key
	{"ur2007", COUNTRY::Ukraine        }, // 102-key
	{"us",     COUNTRY::United_States  }, // Standard
	{"ux",     COUNTRY::United_States  }, // International
	{"uz",     COUNTRY::Uzbekistan     },
	{"vi",     COUNTRY::Vietnam        },
	{"yc",     COUNTRY::Serbia         }, // Deadkey
	{"yc450",  COUNTRY::Serbia         }, // No-deadkey
	{"yu",     COUNTRY::Yugoslavia     },
};

const char *DOS_GetLoadedLayout(void);
void DOS_SetCountry(uint16_t countryNo);
void DOS_KeyboardLayout_Init(Section* sec) {
	test = new DOS_KeyboardLayout(sec);
	sec->AddDestroyFunction(&DOS_KeyboardLayout_ShutDown,true);
	// MAPPER_AddHandler(switch_keyboard_layout, SDL_SCANCODE_F2,
	//                   MMOD1 | MMOD2, "sw_layout", "Switch Layout");

	int countryNo = static_cast<Section_prop*>(sec)->Get_int("country");
	if (!countryNo) {
		const char *layout = DOS_GetLoadedLayout();
		if (layout == NULL)
			countryNo = COUNTRY::United_States;
		else if (country_code_map.find(layout) != country_code_map.end())
			countryNo = country_code_map.find(layout)->second;
	}
	DOS_SetCountry(countryNo);
}
