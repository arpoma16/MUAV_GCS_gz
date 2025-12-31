#!/usr/bin/env python3
"""
Calculate magnetic field vector for a given location
Uses WMM (World Magnetic Model) approximation for simulation

For accurate values, use: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
"""

import math

def calculate_magnetic_field(latitude_deg, longitude_deg, altitude_m=0):
    """
    Calculate magnetic field components for Gazebo simulation

    This is a simplified approximation. For production, use WMM2020 model.

    Args:
        latitude_deg: Latitude in degrees
        longitude_deg: Longitude in degrees
        altitude_m: Altitude in meters above sea level

    Returns:
        tuple: (Bx, By, Bz) in Tesla for Gazebo <magnetic_field> tag
    """

    # Convert to radians
    lat_rad = math.radians(latitude_deg)
    lon_rad = math.radians(longitude_deg)

    # Simplified dipole model approximation
    # Earth's magnetic field at equator: ~3.1e-5 Tesla
    B0 = 3.1e-5  # Tesla

    # Dipole approximation (simplified WMM)
    # North component (X) - points north
    Bx = B0 * math.cos(lat_rad)

    # East component (Y) - points east (very small at most latitudes)
    # Magnetic declination varies by location
    # For Spain: declination is slightly negative (west)
    declination_deg = -1.0  # Approximate for Sevilla
    By = B0 * math.sin(math.radians(declination_deg))

    # Down component (Z) - points down (negative in NED, positive in ENU which Gazebo uses)
    # Inclination for Sevilla is approximately 53 degrees
    inclination_deg = 53.0
    Bz = -B0 * math.sin(math.radians(inclination_deg))  # Negative because points down

    return Bx, By, Bz


def get_wmm_values(latitude_deg, longitude_deg):
    """
    Pre-calculated WMM2020 values for common locations
    Source: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
    """

    locations = {
        # Sevilla, Spain (37.41°N, 6.00°W)
        'sevilla': {
            'lat': 37.41,
            'lon': -6.00,
            'declination': -0.89,  # degrees west
            'inclination': 52.82,   # degrees down
            'total_intensity': 43896.4,  # nT
            'north': 26442.3,  # nT
            'east': -411.0,    # nT
            'down': 35035.6    # nT
        },
        # Zurich, Switzerland (default PX4 world)
        'zurich': {
            'lat': 47.40,
            'lon': 8.55,
            'declination': 2.58,
            'inclination': 64.28,
            'total_intensity': 48279.7,
            'north': 20976.5,
            'east': 944.8,
            'down': 43508.4
        },
        'vienna do castelo': {
            'lat': 41.686629,
            'lon': -8.834516,
            'declination': 	-1.0184,
            'inclination': 56.3374,
            'total_intensity': 45313.1,
            'north': 25117.1,
            'east': -446.2,
            'down': 37714.5
            },
        # San Francisco, USA
        'san_francisco': {
            'lat': 37.77,
            'lon': -122.42,
            'declination': 13.67,
            'inclination': 61.04,
            'total_intensity': 48723.1,
            'north': 23752.7,
            'east': 11520.1,
            'down': 42552.8
        }
    }

    # Find closest location
    min_dist = float('inf')
    closest = None

    for name, loc in locations.items():
        dist = math.sqrt((loc['lat'] - latitude_deg)**2 + (loc['lon'] - longitude_deg)**2)
        if dist < min_dist:
            min_dist = dist
            closest = name

    if min_dist < 1.0:  # Within 1 degree
        loc = locations[closest]
        # Convert from nT to Tesla
        Bx = loc['north'] * 1e-9
        By = loc['east'] * 1e-9
        Bz = loc['down'] * 1e-9

        return Bx, By, Bz, closest

    return None, None, None, None


def format_for_sdf(Bx, By, Bz):
    """Format magnetic field for Gazebo SDF file"""
    return f"{Bx:.2e} {By:.2e} {Bz:.2e}"


if __name__ == '__main__':
    import sys

    print("=" * 80)
    print("Magnetic Field Calculator for Gazebo Simulation")
    print("=" * 80)

    if len(sys.argv) >= 3:
        lat = float(sys.argv[1])
        lon = float(sys.argv[2])
        alt = float(sys.argv[3]) if len(sys.argv) > 3 else 0
    else:
        # Default to Sevilla
        lat = 37.410366
        lon = -6.002338
        alt = 0

    print(f"\nLocation: {lat}°N, {lon}°E, {alt}m altitude")

    # Try to get WMM values first
    Bx_wmm, By_wmm, Bz_wmm, location_name = get_wmm_values(lat, lon)

    if Bx_wmm is not None:
        print(f"\n✅ Using WMM2020 data for: {location_name.upper()}")
        print(f"\nMagnetic Field Components (ENU frame for Gazebo):")
        print(f"  Bx (North): {Bx_wmm:.6e} Tesla = {Bx_wmm*1e9:.1f} nT")
        print(f"  By (East):  {By_wmm:.6e} Tesla = {By_wmm*1e9:.1f} nT")
        print(f"  Bz (Down):  {Bz_wmm:.6e} Tesla = {Bz_wmm*1e9:.1f} nT")
        print(f"\n📋 For your SDF file, use:")
        print(f"  <magnetic_field>{format_for_sdf(Bx_wmm, By_wmm, Bz_wmm)}</magnetic_field>")

        Bx, By, Bz = Bx_wmm, By_wmm, Bz_wmm
    else:
        print("\n⚠️  Using simplified dipole approximation")
        Bx, By, Bz = calculate_magnetic_field(lat, lon, alt)
        print(f"\nMagnetic Field Components (ENU frame):")
        print(f"  Bx (North): {Bx:.6e} Tesla")
        print(f"  By (East):  {By:.6e} Tesla")
        print(f"  Bz (Down):  {Bz:.6e} Tesla")
        print(f"\n📋 For your SDF file, use:")
        print(f"  <magnetic_field>{format_for_sdf(Bx, By, Bz)}</magnetic_field>")

    print("\n" + "=" * 80)
    print("💡 TIP: For production, get exact values from:")
    print("   https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml")
    print("=" * 80 + "\n")
