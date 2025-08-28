namespace Globe.Tectonics
{
    // This file is just a placeholder to explain the split:
    // PlateSystem.Core.cs      -> fields, lifecycle, reseed, readiness checks
    // PlateSystem.Layers.cs    -> data-layer computations (stress/noise) + adapters
    // PlateSystem.Rendering.cs -> ApplyColorsToDual + per-view coloring
    //
    // The class is partial so Unity still sees a single "PlateSystem" component.
    public partial class PlateSystem { }
}
