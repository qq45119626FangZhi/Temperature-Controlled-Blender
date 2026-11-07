plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.tempstir.blecontrol"
    // 本机已安装 android-36.1 平台（minorApiLevel = 1），与 Android Studio 2026.1.2 / AGP 9.3.0 匹配
    compileSdk {
        version = release(36) {
            minorApiLevel = 1
        }
    }

    defaultConfig {
        applicationId = "com.tempstir.blecontrol"
        minSdk = 26
        targetSdk = 33
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            optimization {
                enable = false
            }
        }
    }

    // 纯 Java 源码；列表使用 AndroidX RecyclerView
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_21
        targetCompatibility = JavaVersion.VERSION_21
    }

    // MVVM：开启 ViewBinding，布局通过生成的 *Binding 访问，去掉 findViewById
    buildFeatures {
        viewBinding = true
    }
}

dependencies {
    implementation(libs.androidx.appcompat)
    implementation(libs.androidx.recyclerview)
    // MVVM：ViewModel + LiveData
    implementation("androidx.lifecycle:lifecycle-viewmodel:2.6.1")
    implementation("androidx.lifecycle:lifecycle-livedata:2.6.1")
    // 本机离线缓存中可用的 AndroidX 版本，显式指定避免拉取不存在的传递依赖
    implementation("androidx.core:core:1.10.1")
    implementation("androidx.annotation:annotation:1.6.0")
    implementation("androidx.customview:customview:1.1.0")
    implementation("androidx.collection:collection:1.1.0")
    implementation("androidx.lifecycle:lifecycle-runtime:2.6.1")
    implementation("androidx.lifecycle:lifecycle-common:2.6.1")
}
