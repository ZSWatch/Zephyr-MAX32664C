pluginManagement {
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        maven("https://jitpack.io")
    }
}

rootProject.name = "BLEHeartRateEval"
include(":app")

// Include only required Nordic modules directly to avoid importing their example apps.
include(":ble")
project(":ble").projectDir = file("reference_code/Android-BLE-Library/ble")
include(":ble-ktx")
project(":ble-ktx").projectDir = file("reference_code/Android-BLE-Library/ble-ktx")
