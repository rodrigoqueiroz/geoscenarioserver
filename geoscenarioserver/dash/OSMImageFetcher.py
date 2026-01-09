#!/usr/bin/env python3
# mantkiew@uwaterloo.ca
# --------------------------------------------
# OpenStreetMap Image Fetcher
# Fetches and caches OSM rendered images for map-less scenarios
# Uses direct tile downloading and stitching for compatibility
# --------------------------------------------

import os
import hashlib
import math
import urllib.request
import urllib.error
from typing import Optional, Tuple
from PIL import Image
from io import BytesIO
from lanelet2.core import BasicPoint3d

import logging
log = logging.getLogger(__name__)


def get_default_cache_dir() -> str:
    """Get the default cache directory inside GSS_OUTPUTS."""
    outputs_dir = os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs"))
    return os.path.join(outputs_dir, "osm_cache")


def deg2num(lat_deg: float, lon_deg: float, zoom: int) -> Tuple[float, float]:
    """Convert lat/lon to tile coordinates at given zoom level."""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = (lon_deg + 180.0) / 360.0 * n
    ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return (xtile, ytile)


def num2deg(xtile: float, ytile: float, zoom: int) -> Tuple[float, float]:
    """Convert tile coordinates to lat/lon at given zoom level."""
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)


def get_zoom_for_bbox(min_lat: float, min_lon: float, max_lat: float, max_lon: float, 
                       target_pixels: int = 800) -> int:
    """Calculate appropriate zoom level to fit bbox in target pixel size."""
    # Try different zoom levels to find best fit
    for zoom in range(18, 0, -1):
        x1, y1 = deg2num(max_lat, min_lon, zoom)
        x2, y2 = deg2num(min_lat, max_lon, zoom)
        
        # Calculate size in tiles (each tile is 256 pixels)
        width_pixels = abs(x2 - x1) * 256
        height_pixels = abs(y2 - y1) * 256
        
        # If this fits reasonably in our target, use it
        if width_pixels <= target_pixels and height_pixels <= target_pixels:
            return zoom
    
    return 14  # Default fallback


def download_tile(zoom: int, x: int, y: int, tile_server: str = "https://tile.openstreetmap.org", 
                  cache_dir: Optional[str] = None, timeout: int = 10) -> Optional[Image.Image]:
    """
    Download a single OSM tile.
    
    Args:
        zoom: Zoom level
        x: Tile X coordinate
        y: Tile Y coordinate
        tile_server: OSM tile server URL
        cache_dir: Optional cache directory for tiles
        timeout: HTTP timeout in seconds
        
    Returns:
        PIL Image of the tile, or None if download failed
    """
    # Check cache first
    if cache_dir:
        os.makedirs(os.path.join(cache_dir, "tiles"), exist_ok=True)
        cache_file = os.path.join(cache_dir, "tiles", f"{zoom}_{x}_{y}.png")
        if os.path.exists(cache_file):
            try:
                return Image.open(cache_file)
            except Exception as e:
                log.warning(f"Failed to load cached tile: {e}")
    
    # Download tile
    url = f"{tile_server}/{zoom}/{x}/{y}.png"
    
    try:
        request = urllib.request.Request(url)
        request.add_header('User-Agent', 'GeoScenarioServer/2.0 (educational research tool)')
        
        with urllib.request.urlopen(request, timeout=timeout) as response:
            tile_data = response.read()
            tile_image = Image.open(BytesIO(tile_data))
            
            # Save to cache
            if cache_dir and cache_file:
                tile_image.save(cache_file, 'PNG')
            
            return tile_image
            
    except Exception as e:
        log.warning(f"Failed to download tile {zoom}/{x}/{y}: {e}")
        return None


def calculate_bbox_from_origin(lat: float, lon: float, area_meters: float, projector) -> Tuple[float, float, float, float]:
    """
    Calculate GPS bounding box from origin and area using the projector.
    
    Args:
        lat: Origin latitude (WGS84)
        lon: Origin longitude (WGS84)
        area_meters: Width/height of square area in meters
        projector: LocalCartesianProjector or UtmProjector instance
    
    Returns:
        Tuple of (min_lon, min_lat, max_lon, max_lat)
    """
    half_area = area_meters / 2.0
    
    # Calculate corner points in local coordinates and convert back to GPS
    # Bottom-left corner
    bl_point = projector.reverse(BasicPoint3d(-half_area, -half_area, 0))
    
    # Top-right corner  
    tr_point = projector.reverse(BasicPoint3d(half_area, half_area, 0))
    
    # Bottom-right corner (for better bbox)
    br_point = projector.reverse(BasicPoint3d(half_area, -half_area, 0))
    
    # Top-left corner (for better bbox)
    tl_point = projector.reverse(BasicPoint3d(-half_area, half_area, 0))
    
    # Find min/max from all corners to handle rotation/skew
    all_lons = [bl_point.lon, tr_point.lon, br_point.lon, tl_point.lon]
    all_lats = [bl_point.lat, tr_point.lat, br_point.lat, tl_point.lat]
    
    min_lon = min(all_lons)
    max_lon = max(all_lons)
    min_lat = min(all_lats)
    max_lat = max(all_lats)
    
    return (min_lon, min_lat, max_lon, max_lat)


def calculate_osm_scale(area_meters: float, figure_width_inches: float = 8.0, dpi: float = 100.0) -> float:
    """
    Calculate OpenStreetMap scale parameter (meters per pixel).
    
    The scale parameter determines how many meters each pixel represents.
    Lower values = more detail (more pixels per meter)
    Higher values = less detail (fewer pixels per meter)
    
    Args:
        area_meters: Width/height of area in meters
        figure_width_inches: Width of matplotlib figure in inches
        dpi: Dots per inch of the display
    
    Returns:
        Scale value (meters per pixel), typically 0.1 to 10.0
    """
    # Calculate pixel width of the figure
    figure_width_pixels = figure_width_inches * dpi
    
    # Calculate scale: total area width divided by pixel width
    # Use 2*area_meters because area is half-width (from center)
    scale = (2 * area_meters) / figure_width_pixels
    
    # Clamp to reasonable range for clarity
    # Min 0.1 m/pixel (high detail), Max 10.0 m/pixel (low detail)
    scale = max(0.1, min(10.0, scale))
    
    return scale


def generate_cache_key(lat: float, lon: float, area: float, scale: float) -> str:
    """
    Generate a unique cache key for the image parameters.
    
    Args:
        lat: Origin latitude
        lon: Origin longitude
        area: Area size in meters
        scale: OSM scale parameter (meters per pixel)
    
    Returns:
        Hash string for cache filename
    """
    # Create string representation of parameters
    params_str = f"{lat:.6f}_{lon:.6f}_{area:.1f}_{scale:.3f}"
    
    # Generate hash
    hash_obj = hashlib.md5(params_str.encode())
    return hash_obj.hexdigest()[:8]


def get_cache_filepath(cache_dir: str, lat: float, lon: float, area: float, scale: float) -> str:
    """
    Get the full path for a cached image file.
    
    Args:
        cache_dir: Directory to store cached images
        lat: Origin latitude
        lon: Origin longitude
        area: Area size in meters
        scale: OSM scale parameter
    
    Returns:
        Full path to cache file
    """
    cache_hash = generate_cache_key(lat, lon, area, scale)
    filename = f"osm_{lat:.4f}_{lon:.4f}_{int(area)}_{cache_hash}.png"
    return os.path.join(cache_dir, filename)


def fetch_osm_image(
    lat: float,
    lon: float, 
    area_meters: float,
    projector,
    figure_width_inches: float = 8.0,
    dpi: float = 100.0,
    cache_dir: Optional[str] = None,
    timeout: int = 10
) -> Optional[tuple]:
    """
    Fetch OpenStreetMap rendered image for the given area using direct tile stitching.
    
    This function:
    1. Calculates the bounding box from origin and area
    2. Determines appropriate zoom level
    3. Checks cache for existing stitched image
    4. Downloads and stitches OSM tiles if not cached
    5. Saves to cache for future use
    6. Returns PIL Image object and GPS bounding box
    
    Args:
        lat: Origin latitude (WGS84)
        lon: Origin longitude (WGS84)
        area_meters: Width/height of square area in meters
        projector: LocalCartesianProjector or UtmProjector instance
        figure_width_inches: Width of matplotlib figure in inches
        dpi: Dots per inch of the display
        cache_dir: Directory to store cached images (created if doesn't exist)
        timeout: HTTP request timeout in seconds
    
    Returns:
        Tuple of (PIL Image, (min_lon, min_lat, max_lon, max_lat)) if successful, None if failed
    """
    try:
        # Calculate bounding box
        min_lon, min_lat, max_lon, max_lat = calculate_bbox_from_origin(
            lat, lon, area_meters, projector
        )
        
        # Calculate scale (used for cache key)
        scale = calculate_osm_scale(area_meters, figure_width_inches, dpi)
        
        # Calculate target pixel dimensions
        target_pixels = int(figure_width_inches * dpi)
        
        log.debug(f"OSM Image Request: bbox=({min_lon:.6f},{min_lat:.6f},{max_lon:.6f},{max_lat:.6f})")
        
        # Use default cache directory if not specified
        if cache_dir is None:
            cache_dir = get_default_cache_dir()
        
        # Check cache for stitched image
        cache_filepath = None
        if cache_dir:
            os.makedirs(cache_dir, exist_ok=True)
            cache_filepath = get_cache_filepath(cache_dir, lat, lon, area_meters, scale)
            
            if os.path.exists(cache_filepath):
                log.info(f"Using cached OSM image: {os.path.basename(cache_filepath)}")
                return (Image.open(cache_filepath), (min_lon, min_lat, max_lon, max_lat))
        
        # Determine appropriate zoom level
        zoom = get_zoom_for_bbox(min_lat, min_lon, max_lat, max_lon, target_pixels)
        
        # Get tile coordinates for bounding box
        x1, y1 = deg2num(max_lat, min_lon, zoom)  # top-left
        x2, y2 = deg2num(min_lat, max_lon, zoom)  # bottom-right
        
        # Convert to integer tile indices
        x_min = int(math.floor(min(x1, x2)))
        x_max = int(math.floor(max(x1, x2)))
        y_min = int(math.floor(min(y1, y2)))
        y_max = int(math.floor(max(y1, y2)))
        
        # Calculate pixel offsets within the tiles
        x_offset = int((min(x1, x2) - x_min) * 256)
        y_offset = int((min(y1, y2) - y_min) * 256)
        
        # Calculate size of stitched image
        num_tiles_x = x_max - x_min + 1
        num_tiles_y = y_max - y_min + 1
        stitched_width = num_tiles_x * 256
        stitched_height = num_tiles_y * 256
        
        log.info(f"Downloading and stitching {num_tiles_x}x{num_tiles_y} tiles at zoom {zoom}...")
        
        # Create blank canvas
        stitched_image = Image.new('RGB', (stitched_width, stitched_height))
        
        # Download and stitch tiles
        tiles_downloaded = 0
        for ty in range(y_min, y_max + 1):
            for tx in range(x_min, x_max + 1):
                tile = download_tile(zoom, tx, ty, cache_dir=cache_dir, timeout=timeout)
                if tile:
                    # Calculate position in stitched image
                    paste_x = (tx - x_min) * 256
                    paste_y = (ty - y_min) * 256
                    stitched_image.paste(tile, (paste_x, paste_y))
                    tiles_downloaded += 1
        
        if tiles_downloaded == 0:
            log.warning("Failed to download any tiles")
            return None
        
        # Crop to exact bbox
        crop_width = int(abs(x2 - x1) * 256)
        crop_height = int(abs(y2 - y1) * 256)
        cropped_image = stitched_image.crop((
            x_offset,
            y_offset,
            x_offset + crop_width,
            y_offset + crop_height
        ))
        
        # Resize to target dimensions if needed
        if crop_width != target_pixels or crop_height != target_pixels:
            final_image = cropped_image.resize((target_pixels, target_pixels), Image.Resampling.LANCZOS)
        else:
            final_image = cropped_image
        
        # Save to cache
        if cache_dir and cache_filepath:
            final_image.save(cache_filepath, 'PNG')
            log.info(f"Saved OSM image to cache: {os.path.basename(cache_filepath)}")
        
        log.info(f"Successfully created {final_image.size[0]}x{final_image.size[1]} OSM map")
        return (final_image, (min_lon, min_lat, max_lon, max_lat))
        
    except Exception as e:
        log.error(f"Error fetching OSM image: {str(e)}", exc_info=True)
        return None


def clear_cache(cache_dir: str, older_than_days: Optional[int] = None) -> int:
    """
    Clear cached OSM images.
    
    Args:
        cache_dir: Directory containing cached images
        older_than_days: Only delete files older than this many days (None = delete all)
    
    Returns:
        Number of files deleted
    """
    if not os.path.exists(cache_dir):
        return 0
    
    import time
    deleted_count = 0
    current_time = time.time()
    
    for filename in os.listdir(cache_dir):
        if not filename.startswith('osm_') or not filename.endswith('.png'):
            continue
            
        filepath = os.path.join(cache_dir, filename)
        
        # Check age if specified
        if older_than_days is not None:
            file_age_days = (current_time - os.path.getmtime(filepath)) / (24 * 3600)
            if file_age_days < older_than_days:
                continue
        
        try:
            os.remove(filepath)
            deleted_count += 1
        except Exception as e:
            log.warning(f"Failed to delete cache file {filename}: {e}")
    
    log.info(f"Cleared {deleted_count} cached OSM images from {cache_dir}")
    return deleted_count
