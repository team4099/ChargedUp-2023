package com.team4099.lib.annotations

import com.google.auto.service.AutoService
import com.squareup.kotlinpoet.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.io.File
import javax.annotation.processing.AbstractProcessor
import javax.annotation.processing.Processor
import javax.annotation.processing.RoundEnvironment
import javax.lang.model.SourceVersion
import javax.lang.model.element.ElementKind
import javax.lang.model.element.TypeElement


@AutoService(Processor::class)
class AutoLogGenerator: AbstractProcessor() {

  private val LOGGABLE_TYPES_LOOKUP: MutableMap<String, String> = HashMap()
  private val UNLOGGABLE_TYPES_SUGGESTIONS: MutableMap<String, String> = HashMap()

  init {
    LOGGABLE_TYPES_LOOKUP["byte[]"] = "Raw"
    LOGGABLE_TYPES_LOOKUP["boolean"] = "Boolean"
    LOGGABLE_TYPES_LOOKUP["long"] = "Integer"
    LOGGABLE_TYPES_LOOKUP["float"] = "Float"
    LOGGABLE_TYPES_LOOKUP["double"] = "Double"
    LOGGABLE_TYPES_LOOKUP["java.lang.String"] = "String"
    LOGGABLE_TYPES_LOOKUP["boolean[]"] = "BooleanArray"
    LOGGABLE_TYPES_LOOKUP["long[]"] = "IntegerArray"
    LOGGABLE_TYPES_LOOKUP["float[]"] = "FloatArray"
    LOGGABLE_TYPES_LOOKUP["double[]"] = "DoubleArray"
    LOGGABLE_TYPES_LOOKUP["java.lang.String[]"] = "StringArray"

    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Byte[]"] = "byte[]"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Boolean"] = "boolean"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Long"] = "long"
    UNLOGGABLE_TYPES_SUGGESTIONS["int"] = "long"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Integer"] = "long"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Float"] = "float"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Double"] = "double"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Boolean[]"] = "boolean[]"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Long[]"] = "long[]"
    UNLOGGABLE_TYPES_SUGGESTIONS["int[]"] = "long[]"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Integer[]"] = "long[]"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Float[]"] = "float[]"
    UNLOGGABLE_TYPES_SUGGESTIONS["java.lang.Double[]"] = "double[]"
  }
  override fun getSupportedSourceVersion(): SourceVersion {
    return SourceVersion.latestSupported()
  }

  override fun getSupportedAnnotationTypes(): MutableSet<String> {
    return mutableSetOf(AutoLog::class.java.name)
  }


  override fun process(annotations: MutableSet<out TypeElement>?, roundEnv: RoundEnvironment?): Boolean {
    val elementsWithAutoLog = roundEnv?.getElementsAnnotatedWith(AutoLog::class.java)
    if (elementsWithAutoLog!!.isEmpty()){
      return true
    }
    val packageName = ""


    for (element in elementsWithAutoLog){
      val fileName = element.simpleName.toString() + "AutoLogged"
      val objBuilder = TypeSpec.Companion.objectBuilder(fileName).addSuperinterface(LoggableInputs::class).superclass(element.asType().asTypeName())

      val toLogBuilder = FunSpec.builder("toLog").addModifiers(KModifier.OVERRIDE).addParameter("table", LogTable::class.asTypeName().copy(true))
      val fromLogBuilder = FunSpec.builder("fromLog").addModifiers(KModifier.OVERRIDE).addParameter("table", LogTable::class.asTypeName().copy(true))


      element.enclosedElements.stream().filter { f -> f.kind.equals(ElementKind.FIELD) }.forEach { fieldElement ->
        val simpleName: String = fieldElement.simpleName.toString()
        val unitType = fieldElement.asType()
        val logName = simpleName.substring(0, 1).toUpperCase() + simpleName.substring(1)
        val fieldType: String = fieldElement.asType().toString()
        val logType: String? = LOGGABLE_TYPES_LOOKUP[fieldType]
        println("$simpleName: $fieldType, $unitType")
        if (logType == null) {
          val typeSuggestion: String? = UNLOGGABLE_TYPES_SUGGESTIONS[fieldType]
          val extraText = if (typeSuggestion != null) {
            "Did you mean to use \"$typeSuggestion\" instead?"
          } else {
            "\"$fieldType\" is not supported"
          }
          System.err.println(
            "[AutoLog] Unkonwn type for \"" + simpleName + "\" from \"" + element.simpleName
              + " (" + extraText + ")"
          )
        } else {
          val getterName = "get$logType"
          toLogBuilder.addCode("table?.put(\"$logName\", $simpleName)\n")
          fromLogBuilder.addCode(
            "table?.$getterName(\"$logName\", $simpleName)?.let { $simpleName = it }\n"
          )
          // Need to deep copy arrays
        }
      }
      objBuilder.addFunction(toLogBuilder.build())
      objBuilder.addFunction(fromLogBuilder.build())

      // creating generated file
      val file = FileSpec.builder(packageName, fileName).addType(objBuilder.build()).build()
      val generatedDirectory = processingEnv.options[KAPT_KOTLIN_GENERATED_OPTION_NAME]
      file.writeTo(File(generatedDirectory, "$fileName.kt"))
    }

    return true
  }

  companion object{
    const val KAPT_KOTLIN_GENERATED_OPTION_NAME = "kapt.kotlin.generated"
  }
}
