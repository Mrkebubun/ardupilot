

#include <cstdio>
#include <cmath>
#include <quan/min.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_GPS/AP_GPS.h>
#include <quan/uav/osd/api.hpp>
#include "bitmaps.hpp"
#include "fonts.hpp"

using namespace quan::uav::osd;

void AP_OSD::draw_homeinfo(dequeue::osd_info_t const & info,OSD_params const & osd)
{
    pxp_type pos       {osd.homeinfo_pos.x,
      (( get_video_mode() == video_mode::pal)?osd.homeinfo_pos.y:osd.homeinfo_pos.z)};
    font_ptr font = get_font(Quan::FontID::MWOSD);
    size_type font_size = get_size(font);
    bitmap_ptr home_image = get_bitmap(Quan::BitmapID::home_image);
    size_type home_image_size = get_size(home_image);
    draw_bitmap(home_image,pos);

    pos.x += home_image_size.x + 5;
    // distance to home
    // altitude rel home
    quan::length_<float>::m alt_rel_home = info.aircraft_position.alt - info.home_position.alt;
    char buf[100];
    sprintf(buf,"afh: %8.3f m",static_cast<double>(alt_rel_home.numeric_value()));
    draw_text(buf,pos,font);
    pos.y += font_size.y;
    sprintf(buf,"dth: %8.3f m",static_cast<double>(info.distance_from_home.numeric_value()));
    draw_text(buf,pos,font);
}
