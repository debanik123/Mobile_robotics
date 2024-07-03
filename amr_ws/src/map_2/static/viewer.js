class MapConverter {
    constructor(center, scale) {
        this.center = center;
        this.scale = scale;
    }

    fixedToScreen(coords) {
        return {
            x: (coords.x - this.center.x) * this.scale + window.innerWidth / 2,
            y: (-coords.y - this.center.y) * this.scale + window.innerHeight / 2,
            z: 0
        };
    }

    screenToFixed(coords) {
        return {
            x: (coords.x - window.innerWidth / 2) / this.scale + this.center.x,
            y: -(coords.y - window.innerHeight / 2) / this.scale - this.center.y,
            z: 0
        };
    }

    getPixelsInMapUnits(length) {
        let p1 = this.screenToFixed({ x: 0, y: 0 });
        let p2 = this.screenToFixed({ x: length, y: 0 });
        return Math.abs(p1.x - p2.x);
    }

    getMapUnitsInPixels(length) {
        let p1 = this.fixedToScreen({ x: 0, y: 0 });
        let p2 = this.fixedToScreen({ x: length, y: 0 });
        return Math.abs(p1.x - p2.x);
    }
}

// Create an instance of MapConverter

// console.log(mapConverter.getMapUnitsInPixels(100));
