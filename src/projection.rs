/// Earth radius for Web Mercator (WGS-84 semi-major axis).
const R: f64 = 6_378_137.0;

/// Maximum latitude supported by Web Mercator (avoids ±∞ at poles).
const MAX_WEB_MERCATOR_LAT_DEG: f64 = 85.051_128_779_806_6;

/// Convert geographic coordinates to Web Mercator (EPSG:3857) in meters.
pub fn latlon_to_world(lat_deg: f64, lon_deg: f64) -> [f64; 2] {
    let lat_deg = lat_deg.clamp(-MAX_WEB_MERCATOR_LAT_DEG, MAX_WEB_MERCATOR_LAT_DEG);
    let x = lon_deg.to_radians() * R;
    let y = (std::f64::consts::FRAC_PI_4 + lat_deg.to_radians() / 2.0)
        .tan()
        .ln()
        * R;
    [x, y]
}

/// Convert Web Mercator meters back to geographic coordinates.
#[allow(dead_code)]
pub fn world_to_latlon(x: f64, y: f64) -> [f64; 2] {
    let lon_deg = (x / R).to_degrees();
    let lat_rad = 2.0 * (y / R).exp().atan() - std::f64::consts::FRAC_PI_2;
    let lat_deg = lat_rad.to_degrees();
    [lat_deg, lon_deg]
}
